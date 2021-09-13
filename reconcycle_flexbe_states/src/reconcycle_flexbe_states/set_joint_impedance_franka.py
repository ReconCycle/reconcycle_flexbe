#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *


class SetJointImpedanceProxyClient(EventState):

    '''
    FlexBe state joint impedance Proxy client

    Sets the impedance for each joint in the internal controller.

    User-provided torques are not affected by this setting.
    
    Parameters as follow

    -- joint_stiffness      float[7]    Joint impedance values
    -- robot_name           string      "panda_1" or "panda_2"

    <= continue
    <= failed

    '''
    
    def __init__(self, joint_stiffness, robot_name):
        super(SetJointImpedanceProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.stiffness = joint_stiffness
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        # using franka_msgs.srv.SetJointImpedance
        self.topic = "/" + str(self.robot_name) + "/franka_control/set_joint_impedance"
        self.client_proxy = ProxyServiceCaller({self.topic: SetJointImpedance})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client Set JointImpedance...")

        request = SetJointImpedanceRequest()
        request.joint_stiffness = self.stiffness

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

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        Logger.loginfo("Finished service client!")
        return 'continue'
    
    def on_start(self):
        pass

    def on_stop(self):
        pass