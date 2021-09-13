#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from flexbe_core import EventState, Logger
import robot_module_msgs.msg
import actionlib
import tf


class ReadTFCartLin(EventState):

    '''
    Implements a state that reads cart_lin_action_server data
    [...]

    --	target_frame	string		Transform to this frame
    --	source_frame	string		Transform from this frame
    >#  offset  list                move to x,y,z position
    >#  rotation list               rotate x,y,z in rad

    #< t2_data  Pose()              Read a frame from TF2

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self, target_frame, source_frame):
        rospy.loginfo('__init__ callback happened.')
        super(ReadTFCartLin, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['offset', 'rotation'], output_keys = ['t2_data'])

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
        try:
            t_pose = self.buffer.lookup_transform(self.source_frame, self.target_frame, rospy.Time())
            rot = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
            offset_z = PoseStamped(pose=Pose(position=Point(off[0], off[1], off[2]), orientation=Quaternion(rot[0], rot[1], rot[2], rot[3])))
            t_pose_target = tf2_geometry_msgs.do_transform_pose(offset_z, t_pose)
            Logger.loginfo("source_frame: {}".format(self.source_frame))
            Logger.loginfo("target_frame: {}".format(self.target_frame))
            Logger.loginfo("t_pose data: {}".format(t_pose))
            Logger.loginfo("offset_z: {}".format(offset_z))
            Logger.loginfo("target_pose: {}".format(t_pose_target))
            userdata.t2_data = [t_pose_target.pose]
            return 'continue'
        except (tf2_ros.TransformException, tf2_ros.ConnectivityException):
            Logger.loginfo("Failed to execute")
            return 'failed'
	
    def execute(self, userdata):
        return 'continue'  

    def on_exit(self, userdata):
        Logger.loginfo('Exiting read TF (CartLin).')
        return 'continue'
