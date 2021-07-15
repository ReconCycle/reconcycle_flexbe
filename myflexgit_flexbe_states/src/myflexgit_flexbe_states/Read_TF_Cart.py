#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf
from flexbe_core import EventState, Logger
import robot_module_msgs.msg


class ReadTFCart(EventState):
    
    '''
    Implements a state that reads TF data
    [...]       

    -- target_frame  string     Transform to this frame
    -- source_frame  string     Transform from this frame
    
    #< t1_data  Pose()              The transformation in Pose() format

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self, target_frame, source_frame):
        rospy.loginfo('__init__ callback happened.')   
        super(ReadTFCart, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['t1_data'])
        
        # Initialize the TF listener
        self.listener = tf.TransformListener()  

        # Copy parameters
        self.target_frame = target_frame
        self.source_frame = source_frame
        
  
    def on_enter(self, userdata):
        Logger.loginfo("Started reading TF (Cart)...")
        try:
            if self.target_frame and self.source_frame:
                Logger.loginfo("Recived target and source data... ")
            else:
                raise ValueError('No target and source data!')
        except Exception as e:
             Logger.loginfo('Target or source frame not in Pose() structure!')
             return 'failed'
        
    def execute(self, userdata):
        try:            
            #self.listener.waitForTransform(userdata.target1_frame, userdata.source_frame, rospy.Time.now(), rospy.Duration(4.0))
            # (trans, rot) = self.listener.lookupTransform(self.target1_frame, self.source_frame, rospy.Time(0))
            (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time(0))
            Logger.loginfo("target: {}".format(self.target_frame))
            Logger.loginfo("source: {}".format(self.source_frame))
            Logger.loginfo("trans: {}".format(trans))
            Logger.loginfo("rot: {}".format(rot))
        # except (tf.LookupException, tf.ConnectivityException):
        except Exception as e:
            Logger.loginfo("Exception when reading TF:\n{}".format(e))
            return 'failed'

        data = Pose()
        data.position.x = trans[0]
        data.position.y = trans[1]
        data.position.z = trans[2]
        data.orientation.x = rot[0]
        data.orientation.y = rot[1]
        data.orientation.z = rot[2]
        data.orientation.w = rot[3]
        Logger.loginfo("Pose data from TF (Cart): {}".format(data))
        userdata.t1_data = data
        return 'continue'  

    def on_exit(self, userdata):
        Logger.loginfo('Exiting read TF (Cart).')
        return 'continue'