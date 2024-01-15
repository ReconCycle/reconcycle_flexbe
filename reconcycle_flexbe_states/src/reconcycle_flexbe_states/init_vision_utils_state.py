#!/usr/bin/python

from flexbe_core import EventState, Logger
from disassembly_pipeline.utils.vision_utils import VisionUtils


class InitVisionUtilsState(EventState):

    def __init__(self, camera_name = 'basler', camera_detections_topic = '/vision/basler/detections', camera_table_name = None):
        super(InitVisionUtilsState, self).__init__(outcomes = ['continue', 'failed'],
                                                   input_keys = ['vision_utils'],
                                                   output_keys = ['vision_utils'])
        
        self.camera_name = camera_name
        self.camera_detections_topic = camera_detections_topic
        self.camera_table_name = camera_table_name
        
    def on_enter(self, userdata):
        if not hasattr(userdata, 'vision_utils'):
            userdata['vision_utils'] = {}
        
        userdata['vision_utils'][self.camera_name] = VisionUtils(camera_name = self.camera_name,
                                                                 run_mode = 'manual',
                                                                 vision_topic = self.camera_detections_topic,
                                                                 table_name = self.camera_table_name)
        return 'continue'
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass