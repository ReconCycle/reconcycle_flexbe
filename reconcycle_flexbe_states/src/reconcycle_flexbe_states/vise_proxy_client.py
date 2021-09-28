#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from robot_module_msgs.srv import MoveVise, MoveViseRequest
import rospy


class ChangeViseModelPositionClient(EventState):

    '''
    FlexBe state Change Vise Position client
    
    Parameters as follow

    -- vise         string     name of the vise movable part (vise_plate, vise_rotation, vise_slider)
    -- condition    bool       True/False

    <= continue
    <= failed

    '''
    
    def __init__(self, vise, condition):
        super(ChangeViseModelPositionClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.vise = vise
        self.condition = condition
        
        # Topic and ServicerCaller init
        self.topic = "/move_vise_tool/" + str(self.vise)
        self.client_proxy = ProxyServiceCaller({self.topic: MoveVise})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service move vise controller...")

        request = MoveViseRequest()
        request.trigger = self.condition

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

        except Exception as e:
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