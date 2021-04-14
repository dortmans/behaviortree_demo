#!/usr/bin/env python

# -*- coding: utf-8 -*-
#
# Copyright (c) 2011, Antons Rebguns
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
# 
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# Neither the name of the <ORGANIZATION> nor the names of its contributors may
# be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import roslib; roslib.load_manifest('w2_object_manipulation_launch')
import rospy

from actionlib import SimpleActionClient
from actionlib import SimpleActionServer

from w2_object_manipulation_launch.actions_common import ARM_JOINTS
from w2_object_manipulation_launch.actions_common import READY_POSITION
from w2_object_manipulation_launch.actions_common import move_arm_joint_goal

from w2_object_manipulation_launch.msg import ReadyArmAction
from arm_navigation_msgs.msg import MoveArmAction


ACTION_NAME = 'ready_arm'

class ReadyArmActionServer:
    def __init__(self):
        self.move_arm_client = SimpleActionClient('/move_left_arm', MoveArmAction)
        self.ready_arm_server = SimpleActionServer(ACTION_NAME,
                                                   ReadyArmAction,
                                                   execute_cb=self.ready_arm,
                                                   auto_start=False)
                                                   
    def initialize(self):
        rospy.loginfo('%s: waiting for move_left_arm action server', ACTION_NAME)
        self.move_arm_client.wait_for_server()
        rospy.loginfo('%s: connected to move_left_arm action server', ACTION_NAME)
        
        self.ready_arm_server.start()
        
    def ready_arm(self, goal):
        if self.ready_arm_server.is_preempt_requested():
            rospy.loginfo('%s: preempted' % ACTION_NAME)
            self.move_arm_client.cancel_goal()
            self.ready_arm_server.set_preempted()
            
        if move_arm_joint_goal(self.move_arm_client,
                               ARM_JOINTS,
                               READY_POSITION,
                               collision_operations=goal.collision_operations):
            self.ready_arm_server.set_succeeded()
        else:
            rospy.logerr('%s: failed to ready arm, aborting', ACTION_NAME)
            self.ready_arm_server.set_aborted()


if __name__ == '__main__':
    rospy.init_node('ready_arm_action_server', anonymous=True)
    raas = ReadyArmActionServer()
    raas.initialize()
    rospy.spin()

