#!/usr/bin/python

from flexbe_core import EventState, Logger
from disassembly_pipeline.utils.vision_utils import VisionUtils

class InitVisionUtilsState(EventState):

    def __init__(self, camera_name = 'basler', camera_detections_topic = '/vision/basler/detections', camera_table_name = 'table_vision'):
        super(InitVisionUtilsState, self).__init__(outcomes = ['continue', 'failed'],
                                                   input_keys = ['vision_utils'],
                                                   output_keys = ['vision_utils'])
        
        self.camera_name = camera_name
        self.camera_detections_topic = camera_detections_topic
        self.camera_table_name = camera_table_name
        self.out = 'failed'

        
    def on_enter(self, userdata):

        Logger.loginfo(f'entering VisionUtilsState camera_name: {self.camera_name}')

        if not hasattr(userdata, 'vision_utils') or userdata.vision_utils==None:
            userdata.vision_utils = dict()
        
        try:
            userdata.vision_utils[self.camera_name] = VisionUtils(camera_name = self.camera_name,
                                                                    run_mode = 'flexbe',
                                                                    vision_topic = self.camera_detections_topic,
                                                                    table_name = self.camera_table_name)
            self.out = 'continue'
        except Exception as e:
            breakpoint()
            Logger.loginfo("{}".format(e))
            self.out = 'failed'

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass

if __name__ == '__main__':

    from flexbe_core.core.user_data import UserData
    u = UserData(output_keys=['vision_utils'])
    u.vision_utils = None

    import rospy
    rospy.init_node('visionutils', anonymous = True)
    a = InitVisionUtilsState(camera_name = 'basler', camera_detections_topic = '/vision/basler/detections', camera_table_name = 'table_vision')   
    print(a.on_enter(u))
    print(a.execute(u))
    