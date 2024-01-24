#!/usr/bin/env python

import rospy
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
    
    def __init__(self, camera_name):
        super(GetVisionResultState, self).__init__(outcomes = ['continue', 'failed'],
                                                 input_keys = ['vision_utils'],
                                                 output_keys = ['vision_data'])

        self.camera_name = camera_name
    
    def on_enter(self, userdata):

        desired_object = 'firealarm' # TODO change

        vision_utils = userdata.vision_utils[self.camera_name]
        
        vision_utils.wait_for_message()
        try:
        
            return 'continue'
        
        except Exception as e:
            Logger.loginfo("{}".format(e))

    def execute(self, userdata):
        return 'continue'
        
    def on_exit(self, userdata):
        Logger.loginfo("Finished with vision data acquisition!")
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass