#!/usr/bin/env python

import sys
import rospy
from flexbe_core import EventState, Logger
from robot_module_msgs.srv import ToolChanger


class SwitchToolsRVIZ(EventState):

    '''
    Uses ToolChanger.srv  
    
    -- toolname     string  tool name (screwdriver, vacuumgripper, parallelgripper)
    -- groupname    string  ns (group) of tool (same name as tool name)
    -- transform_to string  attach to robot (panda_2/panda_link8)
    -- hold         bool    True or False
    <= continue             Written successfully
    <= failed               Failed
    '''

    def __init__(self, toolname, groupname, transform_to, hold):
        super(SwitchToolsRVIZ, self).__init__(outcomes = ['continue', 'failed'])
        self.tool_name = toolname
        self.group_name = groupname
        self.transform_to = transform_to
        self.hold = hold
           
    def on_enter(self, userdata):
        Logger.loginfo("Started service client")
        return 'continue'

    def execute(self, userdata):
        # execution of rosservice call
        rospy.wait_for_service("/tool_change")
        try:
            change_tool = rospy.ServiceProxy("/tool_change", ToolChanger)
            sendit = change_tool(self.tool_name,self.group_name, self.transform_to, self.hold)            
            Logger.loginfo("Executing rosservice call to /tool_change and waiting for reply ...")
            Logger.loginfo("Reply: {}".format(sendit))
            return 'continue'
        except rospy.ServiceException as e:
            Logger.loginfo("Data is missing or is not in the right format!")
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Successfully called service /tool_change")
        return 'continue'