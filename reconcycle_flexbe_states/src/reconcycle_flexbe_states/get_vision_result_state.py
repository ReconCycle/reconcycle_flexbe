#!/usr/bin/env python

import rospy
import copy
from flexbe_core import EventState, Logger

class GetVisionResultState(EventState):
    '''
    State for selecting a result from VisionUtils Detections_simple.
    [...]       

    -- camera           String      'camera_1' or 'camera_2'
    -- Z_offset         int64       offset in z for picking object from table
    #< vision_data      Pose()      [position, orientation(quaternions)] 

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    def __init__(self, camera_name, object_to_find = 'firealarm' ):
        super(GetVisionResultState, self).__init__(outcomes = ['continue', 'failed'],
                                                 input_keys = ['vision_utils'],
                                                 output_keys = ['detection'])

        self.camera_name = camera_name
        self.object_to_find = object_to_find # What do you want to get out of detections

        self.out = 'failed'
    
    def on_enter(self, userdata):

        pass

    def execute(self, userdata):

        vision_utils = userdata.vision_utils

        try:
            vision_utils.update_detections(timeout = 0.1)
        except Exception as e:
            Logger.loginfo(f"{e}")
        
        detections = copy.deepcopy(vision_utils.detections)

        detected_smoke_detectors = vision_utils.get_particular_class_from_detections(detections = detections, desired_class = 'firealarm')

        if len(detected_smoke_detectors) > 0:
            self.out = 'continue'
        

        return self.out
        
    def on_exit(self, userdata):
        #Logger.loginfo("Finished with vision data acquisition!")
        return self.out

    def on_start(self):
        pass

    def on_stop(self):
        pass