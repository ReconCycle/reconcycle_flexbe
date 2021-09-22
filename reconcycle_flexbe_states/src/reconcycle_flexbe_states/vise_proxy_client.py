#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from robot_module_msgs.srv import SetList, SetListRequest
import rospy


class ChangeViseModelPositionClient(EventState):

    '''
    FlexBe state Unload controller Proxy client
    
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
        self.topic = "/change_transform_matrix/" + str(self.vise)
        self.client_proxy = ProxyServiceCaller({self.topic: SetList})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client unload controller...")

        request = SetListRequest()

        if self.condition == True:
            if self.vise == "vise_plate":
                request.matrix = [0.05,-0.03,0.019,0,0,0]
            #elif self.vise == "vise_rotation":
            #    request.matrix = [-0.125,0.0788,0.1385,180,0,0]
            elif self.vise == "vise_slider":
                request.matrix = [-0.215,0.0015,0.024,0,0,0]
        else:
            if self.vise == "vise_plate":
                request.matrix = [0.05,0.14,0.019,0,0,0]
            #elif self.vise == "vise_rotation":
            #    request.matrix = [-0.125,0.0788,0.1385,0,0,0]            
            elif self.vise == "vise_slider":
                request.matrix = [-0.26,0.0015,0.024,0,0,0]
            
        Logger.loginfo("Vise:{} \n Matrix:{}".format(self.vise, request.matrix))

        try:
            if self.client_proxy.is_available(self.topic):
                if self.vise == "vise_rotation":
                    if self.condition == True:
                        for i in range(180):
                            request.matrix = [-0.125,0.0788,0.1385,i,0,0]
                            success = self.client_proxy.call(self.topic, request)
                            rospy.sleep(0.001)
                    else:
                        for j in range(180,-1,-2):
                            request.matrix = [-0.125,0.0788,0.1385,j,0,0]
                            success = self.client_proxy.call(self.topic, request)
                            rospy.sleep(0.001)                                
                else:
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