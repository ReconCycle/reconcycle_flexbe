#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from robot_module_msgs.srv import MoveVise, MoveViseRequest # using same .srv message since bool is used in both event states
import rospy


class MoveCutterClient(EventState):

    '''
    FlexBe state Move Cutter client
    
    Parameters as follow

    -- condition    bool       True/False

    <= continue
    <= failed

    '''
    
    def __init__(self, condition):
        super(MoveCutterClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.condition = condition
        
        # Topic and ServicerCaller init
        self.topic = "/move_cutter/cutter"
        self.client_proxy = ProxyServiceCaller({self.topic: MoveVise})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client move cutter controller...")

        request = MoveViseRequest()
        request.trigger = self.condition     

        try:
            if self.client_proxy.is_available(self.topic):
                success = self.client_proxy.call(self.topic, request)
                if success.success == True:
                    Logger.loginfo("Successful: {0}".format(success.success))
                    Logger.loginfo("Cutter moved!")
                    return 'continue'
                elif success.success == False:
                    Logger.loginfo("Successful: {0}".format(success.success))
                    return 'failed'
            else:
                Logger.loginfo("Service is not available!")
                return 'failed'

        except Exception as e:
            Logger.loginfo("Error: {}".format(e))
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