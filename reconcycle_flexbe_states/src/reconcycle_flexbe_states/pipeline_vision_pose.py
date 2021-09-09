#!/usr/bin/env python

# The object assigned to output data is the first detected object from multiple objects detected by vision!
# This is temporaray until selection mode/sequence of objects is defined

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json


class VisionPoseData(EventState):
    '''
    Implements a state for pipeline vision server.
    [...]       

    -- camera           String      'camera_1' or 'camera_2'
    -- Z_offset         int64       offset in z for picking object from table
    #< vision_data      Pose()      [position, orientation(quaternions)] 

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    def __init__(self, camera, Z_offset = 0):
        super(VisionPoseData, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['vision_data'])

        self.camera = camera
        self.offset = Z_offset
        self.data_topic = '/' + str(camera) + '/vision_pipeline/data'
        #### temp testing topic #### 
        #self.data_topic = "/camera_1/vision_pipeline/data"
        ############################

        self.data = None
        self.output_data = Pose()
    
    def on_enter(self, userdata):
        Logger.loginfo("Started object data acquisition, waiting for vision...")
        # define subscriber for data, rospy subscriber
        self.data = rospy.wait_for_message(self.data_topic, String)
        Logger.loginfo("Reading data...")

        try:
            #Logger.loginfo("TOPIC DATA: {}".format(self.data))
            # load JSON String data to load_data and get the info
            loaded_data = json.loads(self.data.data)
            #Logger.loginfo("Loaded data: {}".format(loaded_data))
            class_name = loaded_data[0]['class_name']
            Logger.loginfo("Class Name: {}".format(class_name))
            score = loaded_data[0]['score']
            obb_corners = loaded_data[0]['obb_corners'] #corners coordinates
            Logger.loginfo("obb_corners: {}".format(obb_corners))
            obb_center = loaded_data[0]['obb_center']  #center coordinates
            Logger.loginfo("obb_center: {}".format(obb_center))
            obb_rot_quat = loaded_data[0]['obb_rot_quat'] # Quaternion
            Logger.loginfo("obb_rot_quat: {}".format(obb_rot_quat))

            # display /camera_$num/vision_pipeline/data topic data in json format
            Logger.loginfo("center coordinates(x,y): {}".format(obb_center[0:]))
            Logger.loginfo("rotation quaternion: {}".format(obb_rot_quat[0:]))

            # object center position and rotation
            self.output_data.position.x = obb_center[0]
            self.output_data.position.y = obb_center[1]
            self.output_data.position.z = self.offset
            self.output_data.orientation.x = obb_rot_quat[0]
            self.output_data.orientation.y = obb_rot_quat[1]
            self.output_data.orientation.z = obb_rot_quat[2]
            self.output_data.orientation.w = obb_rot_quat[3]

            Logger.loginfo("Vision data in Pose() format: {}".format(self.output_data))

            userdata.vision_data = self.output_data
        
            return 'continue'
        
        except Exception as e:
            Logger.loginfo("{}".format(e))

    def execute(self, userdata):
        return 'continue'
        
    def on_exit(self, userdata):
        Logger.loginfo("Finished with data acquisition!")
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass