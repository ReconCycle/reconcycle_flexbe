#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
import robot_module_msgs.msg
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from sensor_msgs.msg import JointState
import StringIO
import tf
import sys
import time

rospy.init_node('joints_to_mongodb')

namespace = raw_input("Enter the robot namespace (default: 'panda_1'): ") or "panda_1"
entry_name = raw_input("Enter the entry name (required): ") or None

if None in {namespace, entry_name}:
    raise ValueError("Namespace and/or entry name must not be empty!")

msg_store = MessageStoreProxy()
topic = '/panda_1/joint_states'
joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3',
               'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
sub = ProxySubscriberCached({topic: JointState})

joint_state = None
while joint_state is None:
    joint_state = sub.get_last_msg(topic)

try: 
    present_data = msg_store.query_named(str(entry_name), JointState._type)
    Logger.loginfo("\n\nData in DB: {}".format(present_data))
    Logger.loginfo("\n\nNew data: {}".format(joint_state))
    if present_data[0] != None:
        Logger.loginfo("Id {} already exists, WILL OVERWRITE.".format(entry_name))
        raw_input("\nPress enter to confirm saving to MongoDB...\n")
        client = msg_store.update_named(str(entry_name), joint_state)
    else:
        Logger.loginfo("Writing to DB: id {}, joint positions {}.".format(entry_name, joint_state))
        raw_input("\nPress enter to confirm saving to MongoDB...\n")
        client = msg_store.insert_named(str(entry_name), joint_state)
            
    Logger.loginfo("Written successfully!")

except rospy.ServiceException as e:
    Logger.loginfo("MongoDB is not reachable...")
