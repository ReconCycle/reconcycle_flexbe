#!/usr/bin/env python

import sys
import rospy
import actionlib
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from franka_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
import time


class FrankaErrorRecoveryActionProxy(EventState):
    '''
    FlexBe ErrorRecoveryAction client -> No inputs or outputs, only outcomes.

    -- robot_name   string

    <= continue
    <= failed
    '''

    def __init__(self, robot_name):
        super(FrankaErrorRecoveryActionProxy, self).__init__(outcomes = ['continue', 'failed'])
        
        self.robot_name = robot_name
        self.topic = str(self.robot_name) + "/franka_control/error_recovery"
        Logger.loginfo("Starting franka error recovery action proxy")
        self.recovery_client = ProxyActionClient({self.topic: ErrorRecoveryAction})

    
    def on_enter(self, userdata):
        #-----------------------------------------------------------------------------------------------------------
        goal = ErrorRecoveryActionGoal()

        try:       
            self.recovery_client.send_goal(self.topic, goal)
            time.sleep(0.1)
            Logger.loginfo("Has result?: {}".format(self.recovery_client.has_result(self.topic)))
            if self.recovery_client.has_result(self.topic):
                status = self.recovery_client.get_state(self.topic)
                Logger.loginfo("Status: {}".format(status))
                if status == GoalStatus.SUCCEEDED:
                    Logger.loginfo("Successfully recovered!")
                    return 'continue'
                elif status in [GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.ABORTED]:
                    Logger.logwarn('Action failed: %s' % str(status))
                    return 'failed'
        #-----------------------------------------------------------------------------------------------------------   

        except Exception as e:
            Logger.logwarn("Failed to send a goal: {}".format(str(e)))           
            return 'failed'

    def execute(self, userdata):
        return 'continue'        

    def on_exit(self, userdata):
        if not self.recovery_client.has_result(self.topic):
            self.recovery_client.cancel(self.topic)
            Logger.loginfo("Cancelled active action goal.")

    def on_start(self):
        pass

    def on_stop(self):
        pass