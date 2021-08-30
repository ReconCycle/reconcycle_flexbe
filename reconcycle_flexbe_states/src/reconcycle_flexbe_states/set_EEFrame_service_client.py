#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *


class SetEEFrameProxyClient(EventState):

    '''
    FlexBe state EE Frame Proxy client

    Sets the transformation \(^{NE}T_{EE}\) from nominal end effector to end effector frame.

    The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    
    Parameters as follow

    -- NE_T_EE          float[16]   4x4 matrix -> Vectorized NE-to-EE transformation matrix , column-major.
    -- robot_name       string      "panda_1" or "panda_2"

    <= continue
    <= failed

    '''
    
    def __init__(self, NE_T_EE, robot_name):
        super(SetEEFrameProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.frame = NE_T_EE
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        # using franka_msgs.srv.SetEEFrame
        self.topic = "/" + str(self.robot_name) + "/franka_control/set_EE_frame"
        self.client_proxy = ProxyServiceCaller({self.topic: SetEEFrame})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client Set EEFrame...")        

    def execute(self, userdata):
        request = SetEEFrameRequest()
        request.NE_T_EE = self.frame

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