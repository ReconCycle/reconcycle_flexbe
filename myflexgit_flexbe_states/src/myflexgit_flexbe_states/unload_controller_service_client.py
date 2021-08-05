#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from controller_manager_msgs.srv import *


class UnloadControllerProxyClient(EventState):

    '''
    FlexBe state Unload controller Proxy client
    
    Parameters as follow

    -- desired_controller    string     name of the controller
    -- robot_name            string     name of the robot

    <= continue
    <= failed

    '''
    
    def __init__(self, desired_controller, robot_name):
        super(UnloadControllerProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.controller = desired_controller
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        self.topic = self.robot_name + "/controller_manager/unload_controller"
        self.client_proxy = ProxyServiceCaller({self.topic: UnloadController})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client unload controller...")        

    def execute(self, userdata):
        request = UnloadControllerRequest()
        request.name = self.controller
        try:
            if self.client_proxy.is_available(self.topic):
                success = self.client_proxy.call(self.topic, request)
                if success.ok == True:
                    Logger.loginfo("Successful: {0}".format(success.ok))
                    return 'continue'
                elif success.ok == False:
                    Logger.loginfo("Successful: {0}".format(success.ok))
                    return 'failed'
            else:
                Logger.loginfo("Service is not available!")
                return 'failed'

        except Exception as e:
            Logger.loginfo(e)
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Finished service client!")
        return 'continue'