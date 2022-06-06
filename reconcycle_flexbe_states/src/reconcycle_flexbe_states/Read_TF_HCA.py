#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from flexbe_core import EventState, Logger
from mongodb_store.message_store import MessageStoreProxy
import robot_module_msgs.msg
import actionlib
import tf
import quaternion as qt
import numpy as np


def rotate_ros_quaternion(quat, axis, angle):
    quatty = qt.quaternion(
        quat.pose.orientation.w,
        quat.pose.orientation.x,
        quat.pose.orientation.y,
        quat.pose.orientation.z
    )
    quatty = qt.as_float_array(quatty * get_quat(axis, angle))

    quat.pose.orientation.w = quatty[0]
    quat.pose.orientation.x = quatty[1]
    quat.pose.orientation.y = quatty[2]
    quat.pose.orientation.z = quatty[3]

    return quat


def get_quat(axis, angle):
        if axis == 'x':
            quat = qt.quaternion(np.cos(angle / 2.0), np.sin(angle / 2.0), 0, 0)
        elif axis == 'y':
            quat = qt.quaternion(np.cos(angle / 2.0), 0, np.sin(angle / 2.0), 0)
        elif axis == 'z':
            quat = qt.quaternion(np.cos(angle / 2.0), 0, 0, np.sin(angle / 2.0))
        else:
            raise ValueError('Axis not defined correctly!')
        return quat


class ReadTFHCA(EventState):

    '''
    Implements a state that reads cart_lin_action_server data
    [...]

    --	target_frame	string		 Transform to this frame
    --	source_frame	string		 Transform from this frame

    >#  offset  list                     move to x,y,z position
    >#  rotation list                    rotate x,y,z in rad

    #< t2_data  Pose()                   Read a frame from TF2

    <= continue                          Written successfully
    <= failed                            Failed
    '''

    def __init__(self, target_frame, source_frame):
        rospy.loginfo('__init__ callback happened.')
        super(ReadTFHCA, self).__init__(outcomes = ['continue', 'failed'], 
            input_keys = ['offset', 'rotation'], output_keys = ['t2_data'])

        # Initialize the TF listener
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        rospy.sleep(0.2)

        # Copy parameters
        self.target_frame = target_frame
        self.source_frame = source_frame

    def on_enter(self, userdata):
        Logger.loginfo("Started reading TF (CartLin)...")

        try:
            if self.target_frame and self.source_frame:
                Logger.loginfo("Recived target and source data... ")
            else:
                raise ValueError('No target and source data!')
        except Exception as e:
            Logger.loginfo('Target or source frame not in Pose() structure!')
            return 'failed'
        
        off = userdata.offset
        rot = userdata.rotation
        rot = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
        self.offset = PoseStamped(pose=Pose(position=Point(off[0], off[1], off[2]), 
            orientation=Quaternion(rot[0], rot[1], rot[2], rot[3])))

    def execute(self, userdata):
        try:
            tf_hca = self.buffer.lookup_transform(self.source_frame, self.target_frame, 
                rospy.Time(0), rospy.Duration(10))
            tf_clip = self.buffer.lookup_transform(self.source_frame, "plastic_clip_vision_table_zero", 
                rospy.Time(0), rospy.Duration(10))
            # tf_battery = self.buffer.lookup_transform(self.source_frame, "battery_vision_table_zero", 
            #     rospy.Time(0), rospy.Duration(10))
            tf_hca_target = tf2_geometry_msgs.do_transform_pose(self.offset, tf_hca)
            tf_clip_target = tf2_geometry_msgs.do_transform_pose(self.offset, tf_clip)
            # tf_battery_target = tf2_geometry_msgs.do_transform_pose(self.offset, tf_battery)

            Logger.loginfo("{}".format(tf_hca_target))
            Logger.loginfo("{}".format(tf_clip_target))

            # First rotate HCA frame so that Y axis is towards the robot
            if (tf_hca_target.pose.orientation.z < 0):
                Logger.loginfo("Rotating HCA frame by 180 to point towards robot")
                tf_hca_target = rotate_ros_quaternion(tf_hca_target, 'z', np.pi)
            # Then rotate again depending on plastic clip position
            if (tf_clip_target.pose.position.x - tf_hca_target.pose.position.x) > 0:
                Logger.loginfo("Rotating HCA frame by 180 because of plastic clip")
                tf_hca_target = rotate_ros_quaternion(tf_hca_target, 'z', np.pi)

            Logger.loginfo("source_frame: {}".format(self.source_frame))
            Logger.loginfo("target_frame: {}".format(self.target_frame))
            Logger.loginfo("tf_hca data: {}".format(tf_hca))
            Logger.loginfo("offset: {}".format(self.offset))
            Logger.loginfo("target_pose: {}".format(tf_hca_target))
            userdata.t2_data = [tf_hca_target.pose]
            return 'continue'
        except (tf2_ros.TransformException, tf2_ros.ConnectivityException) as exception:
             Logger.loginfo("Failed to get TF in time!")
             Logger.loginfo("{}".format(exception))
             return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo('Exiting read TF (CartLin).')


if __name__ == '__main__':
     print("Testing standalone")
     rospy.init_node('test_node')
     class UserData:
         offset = [0,0,0]
         rotation = [0,0,0]
     userdata = UserData
     test_state = ReadTFHCA("vision_table_calib", "panda_1/panda_1_link0")
     test_state.on_enter(userdata)
