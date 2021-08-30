#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *


class SetForceTorqueCollisionProxyClient(EventState):

    '''
    FlexBe state Force Torque collision behavior Proxy client

    Changes the collision behavior.

    Set common torque and force boundaries for acceleration/deceleration and constant velocity movement phases.
    
    Parameters as follow

    -- lower_torque_thresholds_nominal  float64[7]      Contact torque thresholds for each joint in [Nm]. 
    -- upper_torque_thresholds_nominal  float64[7]      Collision torque thresholds for each joint in [Nm]. 
    -- lower_force_thresholds_nominal   float64[6]      Contact force thresholds (x,y,z,R,P,Y) for in [N]. 
    -- upper_force_thresholds_nominal   float64[6]      Collision force thresholds (x,y,z,R,P,Y) for in [N]. 
    -- robot_name       string      "panda_1" or "panda_2"

    <= continue
    <= failed

    '''
    
    def __init__(self, lower_torque_thresholds_nominal, upper_torque_thresholds_nominal, lower_force_thresholds_nominal, upper_force_thresholds_nominal, robot_name):
        super(SetForceTorqueCollisionProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.lower_torque_thresholds_nominal = lower_torque_thresholds_nominal
        self.upper_torque_thresholds_nominal = upper_torque_thresholds_nominal
        self.lower_force_thresholds_nominal = lower_force_thresholds_nominal
        self.upper_force_thresholds_nominal = upper_force_thresholds_nominal
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        # using franka_msgs.srv.SetForceTorqueCollisionBehavior
        self.topic = "/" + str(self.robot_name) + "/franka_control/set_force_torque_collision_behavior"
        self.client_proxy = ProxyServiceCaller({self.topic: SetForceTorqueCollisionBehavior})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client Set ForceTorqueCollisionBehavior...")        

    def execute(self, userdata):
        request = SetForceTorqueCollisionBehaviorRequest()
        request.lower_torque_thresholds_nominal = self.lower_torque_thresholds_nominal
        request.upper_torque_thresholds_nominal = self.upper_torque_thresholds_nominal
        request.lower_force_thresholds_nominal = self.lower_force_thresholds_nominal
        request.upper_force_thresholds_nominal = self.upper_force_thresholds_nominal

        try:
            if self.client_proxy.is_available(self.topic):
                success = self.client_proxy.call(self.topic, request)
                if success.success == True:
                    Logger.loginfo("Successful: {0}".format(success.success))
                    return 'continue'
                elif success.success == False:
                    Logger.loginfo("Successful: {0}".format(success.success))
                    return 'failed'
            else:
                Logger.loginfo("Service is not available!")
                return 'failed'

        except (CommandException, NetworkException) as e:
            Logger.loginfo(e)
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Finished service client!")
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass