#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

class GetRosParamState(EventState):
    '''
    A FlexBE state that reads a ROS parameter and outputs its value as 'output_key' data.

    -- param_name    string      The name of the ROS parameter to read.

    ># param_value   string      The value of the ROS parameter.

    <= done                     Successfully read and stored the parameter value.
    <= failed                   Failed to read the parameter.

    '''

    def __init__(self, param_name):
        # Initialize the state
        super(GetRosParamState, self).__init__(outcomes=['done', 'failed'], output_keys=['param_value'])
        self.param_name = param_name

    def on_enter(self, userdata):
        
        # Try to read the ROS parameter
        try:
            param_value = rospy.get_param(self.param_name)
            userdata.param_value = param_value
            Logger.loginfo("Read parameter '{}' with value '{}'".format(self.param_name, param_value))
            self.out = 'done'
        except Exception as e:
            Logger.logwarn("Failed to read parameter '{}': {}".format(self.param_name, str(e)))
            self.out = 'failed'

    def execute(self, userdata):
        return self.out
    

