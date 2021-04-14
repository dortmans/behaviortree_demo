#!/usr/bin/env python
#

"""
Predicate baseclass
"""

##############################################################################
# Imports
##############################################################################

from __future__ import division, absolute_import, print_function
from builtins import super

from abc import ABCMeta, abstractmethod

import rospy
from utils import *
from behaviortree_demo.srv import CheckPredicate, CheckPredicateRequest, CheckPredicateResponse

##############################################################################
# Classes
##############################################################################


class Predicate:
    """
    Predicate service
    """
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name
        self.service_type = CheckPredicate
        self.response = CheckPredicateResponse()

        self.service = rospy.Service(self.name,
                                     self.service_type,
                                     self.execute_cb)

    def execute_cb(self, request):
        """
        Request callback
        """

        self.inputs = zip_dict(goal.input_keys, goal.input_values)

        rospy.loginfo("{name}: start execution".format(name=self.name))

        success = self.execute()

        rospy.loginfo("{name}: stop execution".format(name=self.name))

        if success:
            self.response(success=True)

    @abstractmethod
    def execute(self):
        """
        Execute predicate checking
        """
        pass


##############################################################################
# Test stuff
##############################################################################


if __name__ == '__main__':
    rospy.init_node('predicate_test')

    def on_shutdown():
        rospy.loginfo("{node_name}: shutdown".format(
            node_name=rospy.get_name()))

    rospy.on_shutdown(on_shutdown)

    rospy.spin()
