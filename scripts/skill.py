#!/usr/bin/env python
#

"""
Skill baseclass
"""

##############################################################################
# Imports
##############################################################################

from __future__ import division, absolute_import, print_function
from builtins import super

from abc import ABCMeta, abstractmethod

import rospy
import actionlib
from utils import *
from actionlib_msgs.msg._GoalStatus import GoalStatus
from behaviortree_demo.msg import ApplySkillAction, ApplySkillGoal, ApplySkillResult

##############################################################################
# Classes
##############################################################################

class Skill:
    """
    Skill action
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name
        self.action_type = ApplySkillAction
        self.result = self.action_type().action_result.result
        #self.result = ApplySkillResult()

        self.action_server = actionlib.SimpleActionServer(name,
                                                          self.action_type,
                                                          self.execute_cb,
                                                          auto_start=False)
        self.start()

    def start(self):
        """
        Start the action server.
        """
        self.action_server.start()
        rospy.loginfo("{name}: action server started".format(name=self.name))

    def execute_cb(self, goal):
        """
        Goal callback
        """

        self.inputs = zip_dict(goal.input_keys, goal.input_values)

        # rospy.loginfo("{name}: inputs = {inputs}".format(
        #    name=self.name, inputs=self.inputs))

        self.outputs = {}

        rospy.loginfo("{name}: start execution".format(name=self.name))

        success = self.execute()

        rospy.loginfo("{name}: stop execution".format(name=self.name))

        # rospy.loginfo("{name}: outputs = {outputs}".format(
        #    name=self.name, outputs=self.outputs))

        self.result.output_keys, self.result.output_values = unzip_dict(
            self.outputs)

        if success:
            rospy.loginfo("{name}: succeeded".format(name=self.name))

            self.result.success = True
            self.action_server.set_succeeded(self.result, "succeeded")

    @abstractmethod
    def execute(self):
        """
        Execute skill
        """
        pass


class SkillClient:
    """
    Skill action client
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name
        self.action_type = ApplySkillAction
        self.action_client = actionlib.SimpleActionClient(
            name, self.action_type)
        self.start()

    def start(self):
        """
        Start the action client.
        """
        rospy.loginfo("{name}: waiting for server".format(
            name=self.name))
        if self.action_client.wait_for_server():
            rospy.loginfo("{name}: server ready".format(
                name=self.name))

    def send_goal(self, goal):
        """
        Send goal to action server
        """
        self.action_client.send_goal(goal,
                                     active_cb=None,
                                     feedback_cb=None,
                                     done_cb=self.done_cb)

    def cancel_goal(self):
        """
        Cancel goal on action server.
        """
        if self.action_client.get_state() == GoalStatus.ACTIVE:
            self.action_client.cancel_goal()

    def done_cb(self, state, result):
        rospy.loginfo("{name}: result = {result}".format(
            name=self.name, result=result))

##############################################################################
# Test stuff
##############################################################################

class DummySkill1(Skill):
    def __init__(self):
        super().__init__(name='dummy_skill_1')

    def execute(self):

        success = True

        self.outputs['tau'] = 2*self.inputs['pi']
        self.outputs['out2'] = ['str1', 'str2']
        self.outputs['out3'] = 10
        self.outputs['out4'] = [100, 200]

        return success


class DummySkill2(Skill):
    def __init__(self):
        super().__init__(name='dummy_skill_2')

    def execute(self):
        success = True

        rate = rospy.Rate(10.0)  # hz
        while True:
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                rospy.loginfo("{name}: preempted".format(
                    name=self.name))
                self.action_server.set_preempted()
                success = False
                break
            else:
                # Execute the skill
                pass
            rate.sleep()

        return success


class FibonacciSkill(Skill):
    def __init__(self):
        super().__init__(name='fibonacci_skill')

    def execute(self):
        #order = int(self.inputs['order'])
        order = self.inputs['order']
        success = True

        sequence = []
        sequence.append(0)
        sequence.append(1)

        rate = rospy.Rate(1)
        for i in range(1, order):
            # check that preempt has not been requested by the client
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                rospy.loginfo("{name}: preempted".format(
                    name=self.name))
                self.action_server.set_preempted()
                success = False
                break
            sequence.append(sequence[i] + sequence[i-1])
            rate.sleep()

        if success:
            self.outputs['sequence'] = sequence

        return success


class DummySkill1Client(SkillClient):
    def __init__(self):
        super().__init__(name='dummy_skill_1')


class DummySkill2Client(SkillClient):
    def __init__(self):
        super().__init__(name='dummy_skill_2')


class FibonacciClient(SkillClient):
    def __init__(self):
        super().__init__(name='fibonacci_skill')


if __name__ == '__main__':
    rospy.init_node('skill_test')

    def on_shutdown():
        rospy.loginfo("{node_name}: shutdown".format(
            node_name=rospy.get_name()))

    rospy.on_shutdown(on_shutdown)

    skill1 = DummySkill1()
    skill1_client = DummySkill1Client()

    skill2 = DummySkill2()
    skill2_client = DummySkill2Client()

    fibonacci = FibonacciSkill()
    fibonacci_client = FibonacciClient()

    goal = ApplySkillGoal()
    goal.input_keys.append('pi')
    goal.input_values.append('3.1415927')
    skill1_client.send_goal(goal)

    goal = ApplySkillGoal()
    goal.input_keys.append('param1')
    goal.input_values.append('value')
    goal.input_keys.append('param2')
    goal.input_values.append('value1;value2;value3')
    goal.input_keys.append('param3')
    goal.input_values.append('1.0;2.0;abc')
    skill2_client.send_goal(goal)

    goal = ApplySkillGoal()
    goal.input_keys.append('order')
    goal.input_values.append('5')
    fibonacci_client.send_goal(goal)

    rospy.sleep(1)
    skill1_client.cancel_goal()
    skill2_client.cancel_goal()
    # fibonacci_client.cancel_goal()

    rospy.spin()
