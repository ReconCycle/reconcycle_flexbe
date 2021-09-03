#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
import robot_module_msgs.msg
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, PoseStamped
import StringIO
import tf
import sys
import time

rospy.init_node('pose_to_mongodb')

# source_frame = sys.argv[1]
# target_frame = sys.argv[2]
# entry_name = sys.argv[3]
# print "\nSource frame:", source_frame
# print "Target frame:", target_frame
# print "Entry name:", entry_name

source_frame = raw_input("Enter the source frame (default 'panda_EE'): ") or "panda_EE"
target_frame = raw_input("Enter the target frame (default 'panda_link0'): ") or "panda_link0"
entry_name = raw_input("Enter the entry name (required): ") or None

if entry_name is None:
    raise ValueError("Entry name must not be empty!")

msg_store = MessageStoreProxy()
listener = tf.TransformListener()

listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
(position, rotation) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
pose = Pose()
pose.position.x = position[0]
pose.position.y = position[1]
pose.position.z = position[2]
pose.orientation.x = rotation[0]
pose.orientation.y = rotation[1]
pose.orientation.z = rotation[2]
pose.orientation.w = rotation[3]

try: 
    present_data = msg_store.query_named(str(entry_name), Pose._type)
    Logger.loginfo("\n\nData in DB: {}".format(present_data))
    Logger.loginfo("\n\nNew data: {}".format(pose))
    if present_data[0] != None:
        Logger.loginfo("Id {} already exists, WILL OVERWRITE.".format(entry_name))
        raw_input("\nPress enter to confirm saving to MongoDB...\n")
        client = msg_store.update_named(str(entry_name), pose)
    else:
        Logger.loginfo("Writing to DB: id {}, positon {}, rotation {}.".format(entry_name, position, rotation))
        raw_input("\nPress enter to confirm saving to MongoDB...\n")
        client = msg_store.insert_named(str(entry_name), pose)
            
    Logger.loginfo("Written successfully!")

except rospy.ServiceException as e:
    Logger.loginfo("MongoDB is not reachable...")
