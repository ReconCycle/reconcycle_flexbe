#!/usr/bin/env python

# The object assigned to output data is the first detected object from multiple objects detected by vision!
# This is temporaray until selection mode/sequence of objects is defined

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import json
import numpy as np
import matplotlib.pyplot as plt


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
            for class_num, data in enumerate(loaded_data):
                class_name = data['class_name']
                if 'hca_back' in class_name:
                    break

            Logger.loginfo("Class Name: {}".format(loaded_data[class_num]['class_name']))
            score = loaded_data[class_num]['score']
            obb_corners = loaded_data[class_num]['obb_corners'] #corners coordinates
            #Logger.loginfo("obb_corners: {}".format(obb_corners))
            obb_center = loaded_data[class_num]['obb_center']  #center coordinates
            #Logger.loginfo("obb_center: {}".format(obb_center))
            obb_rot_quat = loaded_data[class_num]['obb_rot_quat'] # Quaternion
            #Logger.loginfo("obb_rot_quat: {}".format(obb_rot_quat))

            #if class_name == 'hca_back':
                #Logger.loginfo("obb_corners: {}".format(obb_corners))

            # display /camera_$num/vision_pipeline/data topic data in json format
            #Logger.loginfo("center coordinates(x,y): {}".format(obb_center[0:]))
            #Logger.loginfo("rotation quaternion: {}".format(obb_rot_quat[0:]))

            # Calculate quaternion from corner coordinates
            distances = []
            corners = np.array(obb_corners)
            Logger.loginfo("Corners: {}".format(corners))
            first_corner = corners[0]

            for ic, corner in enumerate(corners):
                distances.append(np.linalg.norm(corner - first_corner))
            Logger.loginfo("Distances: {}".format(distances))
            distances = np.array(distances)
            idx_edge = distances.argsort()[-2]
            Logger.loginfo("Index of edge: {}".format(idx_edge))
            second_corner = corners[idx_edge]

            highest_y = np.argmax([first_corner[1], second_corner[1]])
            if highest_y == 0:
                vector_1 = first_corner - second_corner
            elif highest_y == 1:
                vector_1 = second_corner - first_corner

            unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
            unit_vector_2 = np.array([0, 1])
            Logger.loginfo("Vectors: {}, {}".format(vector_1, unit_vector_2))
            # dot_product = np.dot(unit_vector_1, unit_vector_2)
            # angle = np.arccos(dot_product)

            angle = (np.arctan2(unit_vector_1[1], unit_vector_1[0]) -
                     np.arctan2(unit_vector_2[1], unit_vector_2[0]))
            # If angle is too negative, add 180 degrees
            if (angle * 180 / np.pi) < -30:
                angle = angle + np.pi
            Logger.loginfo("Angle: {}".format(angle * 180 / np.pi))

            obb_rot_quat = np.concatenate((np.sin(angle/2)*np.array([0,0,1]), 
                                           np.array([np.cos(angle/2)])))
            Logger.loginfo("Quaternion: {}".format(obb_rot_quat))

            # object center position and rotation
            self.output_data.position.x = obb_center[0]
            self.output_data.position.y = obb_center[1]
            self.output_data.position.z = self.offset
            self.output_data.orientation.x = obb_rot_quat[0]
            self.output_data.orientation.y = obb_rot_quat[1]
            self.output_data.orientation.z = obb_rot_quat[2]
            self.output_data.orientation.w = obb_rot_quat[3]

            userdata.vision_data = [self.output_data]
        
            return 'continue'
        
        except Exception as e:
            Logger.logerr("{}".format(e))

    def execute(self, userdata):
        return 'continue'
        
    def on_exit(self, userdata):
        Logger.loginfo("Finished with data acquisition!")
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass