#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *


class SetCartesianImpedanceProxyClient(EventState):

    '''
    FlexBe state cartesian impedance Proxy client

    Sets the Cartesian impedance for (x, y, z, roll, pitch, yaw) in the internal controller.

    User-provided torques are not affected by this setting.
    
    Parameters as follow

    -- cartesian_stiffness  float[6]    (x, y, z, roll, pitch, yaw)
    -- robot_name           string      "panda_1" or "panda_2"

    <= continue
    <= failed

    '''
    
    def __init__(self, cartesian_stiffness, robot_name):
        super(SetCartesianImpedanceProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params init
        self.stiffness = cartesian_stiffness
        self.robot_name = robot_name
        
        # Topic and ServicerCaller init
        # using franka_msgs.srv.SetCartesianImpedance
        self.topic = "/" + str(self.robot_name) + "/franka_control/set_cartesian_impedance"
        self.client_proxy = ProxyServiceCaller({self.topic: SetCartesianImpedance})

    
    def on_enter(self, userdata):
        Logger.loginfo("Started service client Set CartesianImpedance...")

        request = SetCartesianImpedanceRequest()
        request.cartesian_stiffness = self.stiffness

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

    def on_start(self):
        pass

    def on_stop(self):
        pass