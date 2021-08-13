#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *


class SetLoadProxyClient(EventState):

    '''
    FlexBe state Load controller Proxy client

    Sets dynamic parameters of a payload.

    Note
    This is not for setting end effector parameters, which have to be set in the administrator's interface.
    
    Parameters as follow

    -- mass             float64     Mass of the load in [kg]. 
    -- F_x_center_load  float64[3]  Translation from flange to center of mass of load F(xcload) in [m].
    -- load_inertia     float64[9]  Inertia matrix I(load) in [kg x m^2], column-major.
    -- robot_name       string      "panda_1" or "panda_2"

    <= continue
    <= failed

    '''
    
    def __init__(self, mass, F_x_center_load, load_inertia, robot_name):
        super(SetLoadProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.mass = mass
        self.F_x_center_load = F_x_center_load
        self.load_inertia = load_inertia
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        # using franka_msgs.srv.SetLoad
        self.topic = str(self.robot_name) + "/franka_control/set_load"
        self.client_proxy = ProxyServiceCaller({self.topic: SetLoad})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client Set Load...")        

    def execute(self, userdata):
        request = SetLoadRequest()
        request.mass = self.mass
        request.F_x_center_laod = self.F_x_center_load
        request.load_inertia = self.load_inertia

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