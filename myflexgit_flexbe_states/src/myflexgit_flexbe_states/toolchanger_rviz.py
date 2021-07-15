#!/usr/bin/env python

import sys
import rospy
from flexbe_core import EventState, Logger
from robot_module_msgs.srv import rviz_tools


class SwitchToolsRVIZ(EventState):

    '''
    Uses ToolChanger.srv  
    
    -- toolname     string  tool name (screwdriver, vacuumgripper, parallelgripper)
    -- framename    string  attach to robot (panda_2/panda_link8)
    <= continue             Written successfully
    <= failed               Failed
    '''

    def __init__(self, toolname, framename):
        super(SwitchToolsRVIZ, self).__init__(outcomes = ['continue', 'failed'])
        self.tool_name = toolname
        self.frame_name = framename
           
    def on_enter(self, userdata):
        Logger.loginfo("Started service client")
        return 'continue'

    def execute(self, userdata):
        # execution of rosservice call
        frames = "/change_tool_frame/" + str(self.tool_name)
        rospy.wait_for_service(frames)
        try:

            change_tool = rospy.ServiceProxy(frames, rviz_tools)
            sendit = change_tool(self.frame_name)            
            Logger.loginfo("Executing rosservice call to /change_tool_frame/{} and waiting for reply ...".format(self.tool_name))
            Logger.loginfo("Reply: {}".format(sendit))
            return 'continue'
        except rospy.ServiceException as e:
            Logger.loginfo("Data is missing or is not in the right format!")
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Successfully called service /change_tool_frame/{}".format(self.tool_name))
        return 'continue'