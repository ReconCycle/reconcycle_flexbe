#!/usr/bin/python3

import rospy
import actionlib
from flexbe_core import EventState, Logger
import message_filters
from std_msgs.msg import String
from sensor_msgs import Image
import json
import cv_bridge # img to msg converter - not used
import cv2  #add library libopencv-dev or package that includes that library ros-kinetic-opencv3 to flexbe dockerfile


class CameraVision(EventState):
    '''
    Implements a state for pipeline vision server.
    [...]       

    #< vision_image_color   Image       Image from sensor_msgs/Image - OUTPUT
    #< vision_data          String      JSON string from std_msgs.msg/String - OUTPUT 

    <= continue                         Written successfully
    <= failed                           Failed
    '''
    
    def __init(self):
        super(CameraVision, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['vision_image_color', 'vision_data'])


        image_color_topic = '/vision_pipeline/image_color'
        data_topic = '/vision_pipeline/data'

        rospy.init_node('listener', anonymous=True)
        

    def on_enter(self, userdata):
        Logger.loginfo("On enter!")
        # define subscriber for image_color, rospy subscriber
        self.image_sub = rospy.Subscriber(image_color_topic, Image, callback)
        self.data_sub = rospy.Subscriber(data_topic, String, callback)

        return 'continue'


    def execute(self, userdata):
        Logger.loginfo("On execute!")
        image_sub = self.image_sub
        data_sub = self.data_sub
        try:
            # Header is not included.
            if image_sub:
                height = image_sub(height)
                width = image_sub(width)
                encoding = image_sub(encoding)
                is_bigendian = image_sub(is_bigendian)
                step = image_sub(step)
                data_matrix = image_sub(data)

                # display image data transfered into sensor_msgs/Image via cv_bridge on topic /vision_pipeline/image_color
                Logger.loginfo("Image height: {}".format(height))
                Logger.loginfo("Image width: {}".fromat(width))
                Logger.loginfo("encoding: {}".format(encoding))
                Logger.loginfo("is_bigendian: {}".format(is_bigendian))
                Logger.loginfo("step: {}".format(step))
                Logger.loginfo("data_matrix(x,y): {}".format(data_matrix))
                userdata.vision_image_color = image_sub
            else:
                Logger.logdata("No Image data!")
            if data_sub:
                # load JSON String data to load_data and get the info
                loaded_data = json.loads(data_sub)
                class_name = loaded_data['class_name']
                score = loaded_data['score']
                obb_corners = list(loaded_data['obb_corners']) #corners coordinates 
                obb_center = list(loaded_data['obb_center'])  #center coordinates
                obb_rot_quat = list(loaded_data['obb_rot_quat']) # Quaternion

                # display /vision_pipeline/data topic data in json format:
                Logger.loginfo("all_data: {}".format(loaded_data))
                Logger.loginfo("class_name: {}".format(class_name))
                Logger.loginfo("score: {}".format(score))
                Logger.loginfo("corners coordinates(x,y): {}".format(obb_corners))
                Logger.loginfo("center coordinates(x,y): {}".format(obb_center))
                Logger.loginfo("rotation quaternion: {}".format(obb_rot_quat))
                userdata.vision_data = loaded_data
            else:
                Logger.loginfo("No pipeline data!")

        except (ROSException, ROSInterrupException):
            Logger.loginfo("Error in input data!")
            return 'failed'

        # time sync with rospy subscriber
        #ts = message_filters.TimeSynchronizer([image_sub, data_sub], 1)
        #ts.registerCallback(callback)      

        return 'continue'  

    
    def on_exit(self, userdata):
        Logger.loginfo("On exit!")
        return 'continue'