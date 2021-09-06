#!/usr/bin/env python

import sys
import rospy
from flexbe_core import EventState, Logger
from robot_module_msgs.srv import SetList


class AdjustToolPositionAndRotationRVIZ(EventState):

    '''
    Adjust tool postition and rotation [x,y,z,rotx,roty,rotz] in /change_transform_matrix/$(toolname)
    x,y,z are in meters, rotx, roty and rotz in degrees
    Example: rosservice call /change_transform_matrix/screwdriver "matrix: [0,0,0,90,0,0]"  
    
    -- toolname     string  tool name (screwdriver, vacuumgripper, parallelgripper)
    -- matrix       string  matrix input [x(m),y(m),z(m),rotx(deg), roty(deg), rotz(deg)]
    <= continue             Written successfully
    <= failed               Failed
    '''

    def __init__(self, toolname, matrix):
        super(AdjustToolPositionAndRotationRVIZ, self).__init__(outcomes = ['continue', 'failed'])
        self.tool_name = toolname
        self.matrix = matrix
           
    def on_enter(self, userdata):
        Logger.loginfo("Started service client")
        return 'continue'

    def execute(self, userdata):
        # execution of rosservice call
        matrix_service = "/change_transform_matrix/" + str(self.tool_name)
        rospy.wait_for_service(matrix_service)
        try:
            change_tool = rospy.ServiceProxy(matrix_service, SetList)
            sendit = change_tool(self.matrix)            
            Logger.loginfo("Executing rosservice call to /change_transform_matrix/{} and waiting for reply ...".format(self.tool_name))
            Logger.loginfo("Reply: {}".format(sendit))
            return 'continue'
        except rospy.ServiceException as e:
            Logger.loginfo("Data is missing or is not in the right format!")
            return 'failed'

    def on_exit(self, userdata):
        Logger.loginfo("Successfully called service /change_transform_matrix/{}".format(self.tool_name))
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass