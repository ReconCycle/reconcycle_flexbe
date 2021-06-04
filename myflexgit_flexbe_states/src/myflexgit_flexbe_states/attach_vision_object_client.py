#!/usr/bin/env python

import sys
import rospy
from flexbe_core import EventState, Logger
from robot_module_msgs.srv import ToolChanger


class GrabVisionObjectInRVIZ(EventState):

    '''
    Uses ToolChanger.srv  
    
    -- object_tf_prefix     string  object name from /visualization_marker_array
    -- groupname            string  ns (group) of tool (same name as tool name)
    -- transform_to         string  attach to robot (panda_2/panda_link8)
    -- hold                 bool    True or False
    <= continue             Written successfully
    <= failed               Failed
    '''

    def __init__(self, object_tf_prefix, groupname, transform_to, hold):
        super(GrabVisionObjectInRVIZ, self).__init__(outcomes = ['continue', 'failed'])
        self.object_tf_prefix = object_tf_prefix
        self.group_name = groupname
        self.transform_to = transform_to
        self.hold = hold
           
    def on_enter(self, userdata):
        Logger.loginfo("Started service client attach vision object")
        return 'continue'

    def execute(self, userdata):
        # execution of rosservice call
        rospy.wait_for_service("/move_object_with_panda")
        try:
            attach_object = rospy.ServiceProxy("/move_object_with_panda", ToolChanger)
            sendit = attach_object(self.object_tf_prefix,self.group_name, self.transform_to, self.hold)            
            Logger.loginfo("Executing rosservice call to /move_object_with_panda and waiting for reply ...")
            Logger.loginfo("Reply: {}".format(sendit))
            return 'continue'
        except rospy.ServiceException as e:
            Logger.loginfo("Data is missing or is not in the right format!")
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Successfully called service /move_object_with_panda")
        return 'continue'