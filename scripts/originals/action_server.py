#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE

import actionlib
import rospy


class ActionServer(object):
    """
    Generic action server

    Args:
        action_name (:obj:`str`): name of the action server (e.g. move_base)
        action_type (:obj:`any`): type of the action server (e.g. move_base_msgs.msg.MoveBaseAction)
        worker (:obj:`func`): callback to be executed inside the execute loop, no args
        goal_received_callback(:obj:`func`): callback to be executed immediately upon receiving a goal
        duration (:obj:`float`): time for a goal to complete (seconds)
    """
    def __init__(self, action_name, action_type, worker, goal_received_callback=None):
        self.worker = worker
        self.goal_received_callback = goal_received_callback
        self.action_server = actionlib.SimpleActionServer(action_name,
                                                          action_type,
                                                          execute_cb=self.execute,
                                                          auto_start=False
                                                          )
        self.percent_completed = 0
        self.title = action_name.replace('_', ' ').title()
        self.action = action_type()


    def start(self):
        """
        Start the action server.
        """
        self.action_server.start()

    def execute(self, goal):
        """
        Execute action.

        Args:
            goal (:obj:`any`): goal of type specified by the action_type in the constructor.
        """
        if self.goal_received_callback:
            self.goal_received_callback(goal)
        # goal.target_pose = don't care
        frequency = 3.0  # hz
        rate = rospy.Rate(frequency)  # hz
        rospy.loginfo("{title}: received a goal".format(title=self.title))
        # if we just received a goal, we erase any previous pre-emption
        self.action_server.preempt_request = False
        while True:
            if rospy.is_shutdown() or self.action_server.is_preempt_requested():
                rospy.loginfo("{title}: goal preempted".format(title=self.title))
                self.action_server.set_preempted(self.action.action_result.result, "goal was preempted")
                success = False
                break
            else:
                rospy.loginfo("{title}: feedback {percent:.2f}%".format(title=self.title, percent=self.percent_completed))
                self.percent_completed += increment
                self.worker()
            rate.sleep()
        if success:
            rospy.loginfo("{title}: goal success".format(title=self.title))
            self.action_server.set_succeeded(self.action.action_result.result, "goal reached")
