#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String
import json
import numpy as np
#import matplotlib.pyplot as plt


class ReadNextVisionAction(EventState):
    '''
    Implements a state for selecting next vision action.
    [...]       

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    def __init__(self):
        super(ReadNextVisionAction, self).__init__(outcomes = ['move', 'cut', 'lever',
                                                               'turn_over', 'remove_clip'],
                                                   output_keys = ['action'])

        self._topic = '/vision_pipeline_basler/action'
        self._sub = ProxySubscriberCached({self._topic: String})
        self._transition_table = [
            ["plastic_clip", "hca_back", "remove_clip", "remove_clip"],
        ]
    
    def execute(self, userdata):
        if self._sub.has_buffered(self._topic):
            msg = self._sub.get_from_buffer(self._topic)
            detections_str = json.loads(msg.data)
            action = [
                str(detections_str[0]['label']),
                str(detections_str[1]['label']),
                str(detections_str[2])
            ]
            
            Logger.loginfo("Action: {}".format(action))
            for state in self._transition_table:
                Logger.loginfo("{}".format(state))
                if state[0:3] == action:
                    userdata.action = state
                    Logger.loginfo("Setting userdata")
                    return state[3]
			
    def on_enter(self, userdata):
        self._sub.enable_buffer(self._topic)
        self._action = String

    def on_exit(self, userdata):
        self._sub.disable_buffer(self._topic)
        Logger.loginfo("Action: {}".format(self._action))
