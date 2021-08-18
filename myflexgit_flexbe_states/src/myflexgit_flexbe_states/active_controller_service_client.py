#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from controller_manager_msgs.srv import *


class ActiveControllerProxyClient(EventState):

    '''
    FlexBe state Active controller Proxy client
    
    Parameters as follow

    -- robot_name           string     name of the robot
    -- real_controllers     string[]
    #< active_controller    

    <= continue
    <= failed

    '''
    
    def __init__(self, robot_name, real_controllers):
        super(ActiveControllerProxyClient, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['active_controller'])
        
        # Input params init
        self.robot_name = robot_name
        self.real_controllers = real_controllers

        # Topic and ServicerCaller init
        self.topic = "/" + str(self.robot_name) + "/controller_manager/list_controllers"
        self.client_proxy = ProxyServiceCaller({self.topic: ListControllers})

        # Variables declaration
        self.real_controller_list = []
        self.real_controller_status = []

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client active controller...")        

    def execute(self, userdata):
        request = ListControllersRequest()
        try:
            if self.client_proxy.is_available(self.topic):
                controller_list = self.client_proxy.call(self.topic, request)
                
                for controller in controller_list.controller:
                    for real_controller in self.real_controllers:
                        if(controller.name == real_controller):
                            self.real_controller_list.append(controller.name)
                            self.real_controller_status.append(controller.state)

                i = 0
                for status in self.real_controller_status:
                    if(status == "running"):
                        active_controller = self.real_controller_list[i]
                    i += 1
                Logger.loginfo("Active controller: {0}".format(active_controller))
                # Retruning active_controller as output_keys    
                userdata.active_controller = active_controller
                # If successful state continues to next one or finishes successfully
                return 'continue'
            
            else:
                Logger.loginfo("Service is not available!")
                return 'failed'

        except Exception as e:
            Logger.loginfo(e)
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Finished service client!")
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass