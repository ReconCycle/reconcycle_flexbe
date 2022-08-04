#!/usr/bin/env python

import sys
import rospy
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from franka_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
import time


class RBS_FrankaErrorRecoveryActionProxy(EventState):
    '''
    FlexBe ErrorRecoveryAction client -> No inputs or outputs, only outcomes.

    -- robot_name   string

    <= continue
    <= failed
    '''

    def __init__(self, robot_name):
        super(RBS_FrankaErrorRecoveryActionProxy, self).__init__(outcomes = ['continue', 'failed'])
        
        self.robot_name = robot_name

    
    def on_enter(self, userdata):
        #-----------------------------------------------------------------------------------------------------------
        robot = getattr(userdata, self.robot_name)
        robot.error_recovery()
        return 'continue'

    def execute(self, userdata):
        return 'continue'        

    def on_exit(self, userdata):
        0
        #if not self.recovery_client.has_result(self.topic):
        #    self.recovery_client.cancel(self.topic)
        #    Logger.loginfo("Cancelled active action goal.")

    def on_start(self):
        pass

    def on_stop(self):
        pass
