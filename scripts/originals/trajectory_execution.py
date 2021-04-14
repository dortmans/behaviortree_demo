#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('control_msgs')

import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction
from actionlib_msgs.msg import GoalStatus


class TrajectoryExecution:

    def __init__(self, side_prefix):
        
        traj_controller_name = '/' + side_prefix + '_arm_controller/joint_trajectory_action'
        self.traj_action_client = SimpleActionClient(traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server arm: ' + side_prefix)
        self.traj_action_client.wait_for_server()
        rospy.loginfo('Trajectory executor is ready for arm: ' + side_prefix)
    
        motion_request_name = '/' + side_prefix + '_arm_motion_request/joint_trajectory_action'
        self.action_server = SimpleActionServer(motion_request_name, JointTrajectoryAction, execute_cb=self.move_to_joints)
        self.action_server.start()
	self.has_goal = False

    def move_to_joints(self, traj_goal):
        rospy.loginfo('Receieved a trajectory execution request.')
    	traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        self.traj_action_client.send_goal(traj_goal)
	rospy.sleep(1)
	is_terminated = False
	while(not is_terminated):
	    action_state = self.traj_action_client.get_state()
	    if (action_state == GoalStatus.SUCCEEDED):
		self.action_server.set_succeeded()
		is_terminated = True
	    elif (action_state == GoalStatus.ABORTED):
		self.action_server.set_aborted()
		is_terminated = True
	    elif (action_state == GoalStatus.PREEMPTED):
		self.action_server.set_preempted()
		is_terminated = True
	rospy.loginfo('Trajectory completed.')

if __name__ == '__main__':
    rospy.init_node('trajectory_execution')

    right = TrajectoryExecution('r')
    left = TrajectoryExecution('l')

    rospy.spin()

    #while not rospy.is_shutdown():
    #    right.update_status()
    #    left.update_status()


