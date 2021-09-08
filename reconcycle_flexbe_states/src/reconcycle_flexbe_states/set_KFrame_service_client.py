#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *


class SetKFrameProxyClient(EventState):

    '''
    FlexBe state K frame Proxy client

    Sets the transformation \(^{EE}T_K\) from end effector frame to stiffness frame.

    The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    
    Parameters as follow

    -- EE_T_K           float[16]   Vectorized EE-to-K transformation matrix , column-major.
    -- robot_name       string      "panda_1" or "panda_2"

    <= continue
    <= failed

    '''
    
    def __init__(self, EE_T_K, robot_name):
        super(SetKFrameProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.frame = EE_T_K
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        # using franka_msgs.srv.SetKFrame
        self.topic = "/" + str(self.robot_name) + "/franka_control/set_K_frame"
        self.client_proxy = ProxyServiceCaller({self.topic: SetKFrame})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client Set KFrame...")

        request = SetKFrameRequest()
        request.EE_T_K = self.frame

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