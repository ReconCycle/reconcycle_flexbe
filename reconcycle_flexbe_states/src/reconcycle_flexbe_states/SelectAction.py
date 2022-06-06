#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String
import json
import numpy as np
import matplotlib.pyplot as plt


class ReadAction(EventState):
    '''
    Implements a state for selecting action outcome.
    [...]       

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    def __init__(self):
        super(ReadAction, self).__init__(outcomes = ['move', 'cut', 'lever',
                                                     'turn_over', 'remove_clip'],
                                         input_keys = ['action'],
                                         output_keys = ['action_out'])

    def execute(self, userdata):
        action = userdata.action
        return action[-1]
			
    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass