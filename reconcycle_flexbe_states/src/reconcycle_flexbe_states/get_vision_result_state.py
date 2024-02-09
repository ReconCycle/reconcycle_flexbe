#!/usr/bin/env python

import rospy
import copy
import numpy as np
from flexbe_core import EventState, Logger

from disassembly_pipeline.environment.types import SmokeDetector

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
    
    def __init__(self, camera_name, object_to_find = 'firealarm', timeout = 3.5):
        super(GetVisionResultState, self).__init__(outcomes = ['continue', 'failed'],
                                                 input_keys = ['vision_utils'],
                                                 output_keys = ['detection'])

        self.camera_name = camera_name
        self.object_to_find = object_to_find # What do you want to get out of detections
        self.timeout = timeout

        self.allowed_time_per_execute_loop = 0.7 # How long we wait for detecting in each execute() loop
        self.max_n_retries = int(self.timeout / self.allowed_time_per_execute_loop) # Maximum number of execute() loops

        self.out = 'failed'
    
    def on_enter(self, userdata):
        # Reset all variables
        self.out = 'failed'

        userdata.detection = None
        # Variable to keep track of n_retries
        self.n_retries = 0 
        pass

    def execute(self, userdata):

        vision_utils = userdata.vision_utils[self.camera_name] # Get results/vision_utils['basler'] of particular camera
        table_name = vision_utils.table_name
        parent_frame = vision_utils.parent_frame

        self.n_retries +=1
        #print(f"{self.n_retries} / {self.max_n_retries}")

        detections = []
        try:
            vision_utils.update_detections(timeout = self.allowed_time_per_execute_loop)
            detections = copy.deepcopy(vision_utils.detections)
        except Exception as e:
            Logger.logerr(f"{e}")
            # break out of fn if no result.
            return

        # If self.object_to_find is None, we presume we want to to return all detections.
        # TODO BUG: might be we don't get any detections due to timeout
        if self.object_to_find is None:
            detected_objects_of_interests = detections
            userdata.detection = detections
            self.out = 'continue'

        # self.object_to_find is not None, so we actually need at least one detection.
        elif (self.object_to_find is not None) and (len(detections)>0):
            #Logger.loginfo("{}".format(len(detections)))
            detected_objects_of_interests = vision_utils.get_particular_class_from_detections(
                detections = detections, desired_class = self.object_to_find)
            if len(detected_objects_of_interests) > 0:
                # _get_simple_detections will add attributes such as tf_name, general_class, class_and_id, to the detection.
                random_idx = np.random.randint(0, len(detected_objects_of_interests))
                disassembly_object = detected_objects_of_interests[random_idx]

                if 'firealarm' in disassembly_object.general_class:
                    disassembly_object = SmokeDetector(disassembly_object)
                else:
                    0
                    # HCAs etc unhandled as of now.
                    #raise ValueError("Not setting disassembly parameters because of unhandled object type {}".format(disassembly_object.general_class))

                userdata.detection = disassembly_object

                Logger.loginfo("output detection: {}, {}".format(disassembly_object.tf_name, disassembly_object.general_class))
                self.out = 'continue'

        # Return upon getting a result
        if (self.out == 'continue') or (self.n_retries > self.max_n_retries):
            return self.out

    def on_exit(self, userdata):
        #Logger.loginfo("Finished with vision data acquisition!")
        return self.out

    def on_start(self):
        pass

    def on_stop(self):
        pass

# if main
if __name__ == '__main__':
    rospy.init_node('test_detections',anonymous=True)
    from flexbe_core.core.user_data import UserData
    from init_vision_utils_state import InitVisionUtilsState
    ud = UserData(input_keys=['vision_utils', 'detection'], output_keys=['vision_utils', 'detection'])
    ud.vision_utils = None
    vision_state = InitVisionUtilsState(camera_name = 'basler', camera_detections_topic = '/vision/basler/detections', camera_table_name = 'table_vision')
    vision_result_state = GetVisionResultState(camera_name='basler', object_to_find='firealarm',timeout=5)

    print("....")
    vision_state.on_enter(ud)
    #print("....")
    vision_state.execute(ud)
    #print("....")
    vision_result_state.on_enter(ud)
    #print("....")
    #for i in range(0,10):
    vision_result_state.execute(ud)
    #print("....")
    #import ipdb; ipdb.set_trace();
    #print(ud.detection)

