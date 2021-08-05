#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from controller_manager_msgs.srv import *


class SwitchControllerProxyClient(EventState):

    '''
    FlexBe state Switch controller Proxy client
    
    Parameters as follow

    -- robot_name           string     name of the robot
    -- start_controller     string[]   start_controller array  
    -- stop_controller      string[]   stop_controller array
    -- strictness           int32      

    <= continue
    <= failed

    '''
    
    def __init__(self, robot_name, start_controller, stop_controller, strictness):
        super(SwitchControllerProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.start_controller = start_controller
        self.stop_controller = stop_controller
        self.strictness = strictness
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        self.topic = self.robot_name + "/controller_manager/switch_controller"
        self.client_proxy = ProxyServiceCaller({self.topic: SwitchController})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client switch controller...")  

    def execute(self, userdata):
        request = SwitchControllerRequest()
        request.start_controllers = self.start_controller
        request.stop_controllers = self.stop_controller
        request.strictness = self.strictness
        Logger.loginfo("request: {0}".format(request))
        
        #try:
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

        #except Exception as e:
        #    Logger.loginfo(e)
        #    return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Finished service client!")
        return 'continue'