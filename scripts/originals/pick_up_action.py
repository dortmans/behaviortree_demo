#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran

PKG = 'test_manipulation'
NAME = 'pick_up_action'

import roslib; roslib.load_manifest(PKG)
import rospy
import tf

from actionlib import SimpleActionServer
from actionlib import SimpleActionClient

from wubble_actions.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from test_manipulation.msg import *
from std_msgs.msg import Float64
from pr2_controllers_msgs.msg import JointControllerState

import math


class PickUpActionServer():

    def __init__(self):


        # Initialize new node
        rospy.init_node(NAME)#, anonymous=False)

        #initialize the clients to interface with 
        self.arm_client = SimpleActionClient("smart_arm_action", SmartArmAction)
        self.gripper_client = SimpleActionClient("smart_arm_gripper_action", SmartArmGripperAction)
        self.move_client = SimpleActionClient("erratic_base_action", ErraticBaseAction)
        
        self.move_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()

        # Initialize tf listener (remove?)
        self.tf = tf.TransformListener()

        # Initialize erratic base action server
        self.result = SmartArmGraspResult()
        self.feedback = SmartArmGraspFeedback()
        self.server = SimpleActionServer(NAME, SmartArmGraspAction, self.execute_callback)

        #define the offsets
    	# These offsets were determines expirmentally using the simulator
    	# They were tested using points stamped with /map
        self.xOffset = 0.025
        self.yOffset = 0.0
        self.zOffset = 0.12 #.05 # this does work!

        rospy.loginfo("%s: Pick up Action Server is ready to accept goals", NAME)
        rospy.loginfo("%s: Offsets are [%f, %f, %f]", NAME, self.xOffset, self.yOffset, self.zOffset )

    def move_to(self, frame_id, position, orientation, vicinity=0.0):
        goal = ErraticBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.pose.position = position
        goal.target_pose.pose.orientation = orientation
        goal.vicinity_range = vicinity
        self.move_client.send_goal(goal)
        #print "going into loop"
        while (not self.move_client.wait_for_result(rospy.Duration(0.01))):
            # check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.move_client.cancel_goal()
                return GoalStatus.PREEMPTED
        
        return self.move_client.get_state()

    #(almost) blatent copy / past from wubble_head_action.py.  Err, it's going to the wrong position
    def transform_target_point(self, point, frameId):
        
        #rospy.loginfo("%s: got %s %s %s %s", NAME, point.header.frame_id, point.point.x, point.point.y, point.point.z)

        # Wait for tf info (time-out in 5 seconds)
        self.tf.waitForTransform(frameId, point.header.frame_id, rospy.Time(), rospy.Duration(5.0))

        # Transform target point & retrieve the pan angle
        return self.tf.transformPoint(frameId, point)

#UNUSED
    def move_base_feedback_cb(self, fb):
        self.feedback.base_position = fb.base_position
        if self.server.is_active():
            self.server.publish_feedback(self.feedback)

# This moves the arm into a positions based on angles (in rads)
# It depends on the sources code in wubble_actions
    def move_arm(self, shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate):
        goal = SmartArmGoal()
        goal.target_joints = [shoulder_pan, shoulder_tilt, elbow_tilt, wrist_rotate]

        self.arm_client.send_goal(goal, None, None, self.arm_position_feedback_cb)
        self.arm_client.wait_for_goal_to_finish()

        while not self.arm_client.wait_for_result(rospy.Duration(0.01)) :
            # check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.arm_client.cancel_goal()
                return GoalStatus.PREEMPTED

        return self.arm_client.get_state()

# This moves the wrist of the arm to the x, y, z coordinates
    def reach_at(self, frame_id, x, y, z):
	        
        goal = SmartArmGoal()
        goal.target_point = PointStamped()
        goal.target_point.header.frame_id = frame_id
        goal.target_point.point.x = x
        goal.target_point.point.y = y
        goal.target_point.point.z = z

       	rospy.loginfo("%s: Original point is '%s' [%f, %f, %f]", NAME, frame_id,\
		goal.target_point.point.x, goal.target_point.point.y, goal.target_point.point.z) 

        goal.target_point = self.transform_target_point(goal.target_point, '/arm_base_link');

        rospy.loginfo("%s: Transformed point is '/armbaselink' [%f, %f, %f]", NAME, goal.target_point.point.x, \
        goal.target_point.point.y, goal.target_point.point.z)


        goal.target_point.point.x = goal.target_point.point.x + self.xOffset
        goal.target_point.point.y = goal.target_point.point.y + self.yOffset
        goal.target_point.point.z = goal.target_point.point.z + self.zOffset

        rospy.loginfo("%s: Transformed and Offset point is '/armbaselink' [%f, %f, %f]", NAME, goal.target_point.point.x, \
        goal.target_point.point.y, goal.target_point.point.z)

        self.arm_client.send_goal(goal, None, None, self.arm_position_feedback_cb)
        self.arm_client.wait_for_goal_to_finish()

        while not self.arm_client.wait_for_result(rospy.Duration(0.01)) :
            # check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.arm_client.cancel_goal()
                return GoalStatus.PREEMPTED

        return self.arm_client.get_state()

#This method is used passes updates from arm positions actions to the feedback
#of this server
    def arm_position_feedback_cb(self, fb):
        self.feedback.arm_position = fb.arm_position
        if self.server.is_active():
            self.server.publish_feedback(self.feedback)

#This moves the gripper to the given angles
    def move_gripper(self, left_finger, right_finger):
        goal = SmartArmGripperGoal()
        goal.target_joints = [left_finger, right_finger]

        self.gripper_client.send_goal(goal, None, None, self.gripper_position_feedback_cb)
        #gripper_client.wait_for_goal_to_finish()

        while not self.gripper_client.wait_for_result(rospy.Duration(0.01)) :
            # check for preemption
            if self.server.is_preempt_requested():
                rospy.loginfo("%s: Aborted: Action Preempted", NAME)
                self.gripper_client.cancel_goal()
                return GoalStatus.PREEMPTED

        return self.gripper_client.get_state()

#This method is used passes updates from arm positions actions to the feedback
#of this server
    def gripper_position_feedback_cb(self, fb):
        self.feedback.gripper_position = fb.gripper_position
        if self.server.is_active():
            self.server.publish_feedback(self.feedback)

#This method sets the results of the goal to the last feedback values
    def set_results(self, success):
        self.result.success = success
        self.result.arm_position = self.feedback.arm_position
        self.result.gripper_position = self.feedback.gripper_position
	self.server.set_succeeded(self.result)

#This is the callback function that is executed whenever the server 
#recieves a request
    def execute_callback(self, goal):
	
        rospy.loginfo("%s: Executing Grasp Action [%s, %f, %f, %f]", NAME, \
			goal.target_point.header.frame_id, goal.target_point.point.x, \
			goal.target_point.point.y, goal.target_point.point.z)

        rospy.loginfo( "%s: Moving the arm to the cobra pose", NAME)
    	#move the arm into the cobra position
    	result = self.move_arm(0.0, 1.972222, -1.972222, 0.0)
    	if result == GoalStatus.PREEMPTED:	#action has failed
    	    rospy.loginfo("%s: 1st Move Arm (to cobra pose) Preempted", NAME)
    	    self.server.set_preempted()
    	    self.set_results(False)
    	    return		
    	elif result != GoalStatus.SUCCEEDED:
    	    rospy.loginfo("%s: 1st Move Arm (to cobra pose) Failed", NAME)
    	    self.set_results(False)
    	    return

        position = Point(x = goal.target_point.point.x, y = goal.target_point.point.y)
        orientation = Quaternion(w=1.0)
        self.move_to('/map', position, orientation, 0.5)
        self.move_to('/map', position, orientation, 0.2)
        rospy.sleep(0.2)

        rospy.loginfo( "%s: Opening gripper", NAME)
        #open the gripper
        result = self.move_gripper(0.2, -0.2)
        if result == GoalStatus.PREEMPTED:	#action has failed
            rospy.loginfo("%s: Open Gripper Preempted", NAME)
            self.server.set_preempted()
            self.set_results(False)
            return		
        elif result != GoalStatus.SUCCEEDED:
            rospy.loginfo("%s: Open Gripper Failed", NAME)
            self.set_results(False)
            return
            
        rospy.loginfo( "%s: Moving the arm to the object", NAME)
	#move the arm to the correct posisions
        result = self.reach_at(goal.target_point.header.frame_id, \
                goal.target_point.point.x, \
                goal.target_point.point.y, \
                goal.target_point.point.z)
        if result == GoalStatus.PREEMPTED:	#action has failed
            rospy.loginfo("%s: 2nd Move Arm (to object) Preempted", NAME)
            self.server.set_preempted()
            self.set_results(False)
            return		
        elif result != GoalStatus.SUCCEEDED:
            rospy.loginfo("%s: 2nd Move Arm (to object) Failed", NAME)
            self.set_results(False)
            return
	
        rospy.loginfo( "%s: Moving the elbow joint to the cobra pose", NAME)
    	#move the arm into the cobra position
    	result = self.move_arm(self.feedback.arm_position[0], self.feedback.arm_position[1], \
		-3.14 / 2 - self.feedback.arm_position[1], self.feedback.arm_position[3])
    	if result == GoalStatus.PREEMPTED:	#action has failed
    	    rospy.loginfo("%s: Moving the elbow joint Preempted", NAME)
    	    self.server.set_preempted()
    	    self.set_results(False)
    	    return		
    	elif result != GoalStatus.SUCCEEDED:
    	    rospy.loginfo("%s: Moving the elbow joint Failed", NAME)
    	    self.set_results(False)
    	    return

        rospy.loginfo( "%s: Closing gripper", NAME)
        #close the gripper
        result = self.move_gripper(-1.5, 1.5)
	if result == GoalStatus.PREEMPTED:	#action has failed
            rospy.loginfo("%s: Close Gripper Preempted", NAME)
            self.server.set_preempted()
            self.set_results(False)
            return		
        #this actions 'succeeds' if it times out

        rospy.loginfo( "%s: Moving the arm to the cobra pose", NAME)
	#move the arm into the cobra position
	result = self.move_arm(0.0, 1.972222, -1.972222, 0.0)
	if result == GoalStatus.PREEMPTED:	#action has failed
            rospy.loginfo("%s: 3rd Move Arm (to cobra pose) Preempted", NAME)
            self.server.set_preempted()
            self.set_results(False)
            return		
	elif result != GoalStatus.SUCCEEDED:
            rospy.loginfo("%s: 3rd Move Arm (to cobra pose) Failed", NAME)
            self.set_results(False)
            return

        #action has succeeded
        rospy.loginfo("%s: Grasp Action Succeed [%s, %f, %f, %f]", NAME, \
		goal.target_point.header.frame_id, goal.target_point.point.x, \
		goal.target_point.point.y, goal.target_point.point.z)
        self.set_results(True)   
  
	"""
        r = rospy.Rate(100)
        rospy.loginfo("%s: Executing Grasp Action", NAME)
        
        move_base_result = None
        if (goal.vicinity_range == 0.0):
            # go to exactly
            move_base_result = self.move_to(goal.target_pose)
        else:
            # go near (within vicinity_range meters)
            vicinity_target_pose = self.get_vicinity_target(goal.target_pose, goal.vicinity_range)
            move_base_result = self.move_to(vicinity_target_pose)

        # check results
        if (move_base_result == GoalStatus.SUCCEEDED):
            rospy.loginfo("%s: Succeeded", NAME)
            self.result.base_position = self.feedback.base_position
            self.server.set_succeeded(self.result)
        elif (move_base_result == GoalStatus.PREEMPTED):
            rospy.loginfo("%s: Preempted", NAME)
            self.server.set_preempted()
        else:
            rospy.loginfo("%s: Aborted", NAME)
            self.server.set_aborted()
	"""


#The main function
if __name__ == '__main__':
    try:
        w = PickUpActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

