#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String
import json
import numpy as np
import matplotlib.pyplot as plt


class SelectAction(EventState):
    '''
    Implements a state for selecting action outcome.
    [...]       

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    def __init__(self):
        super(SelectAction, self).__init__(outcomes = ['move', 'cut', 'lever',
                                                       'turn_over', 'remove_clip'],
                                           output_keys = ['action'])

    def execute(self, userdata):
        action = userdata.action
        return action[-1]
			
    def on_enter(self, userdata):
        pass

    def on_exit(self, userdata):
        pass