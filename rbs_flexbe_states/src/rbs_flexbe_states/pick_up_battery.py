#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

#from sensor_msgs.msg import JointState

import time
import numpy as np
import sys
import rospy

from disassembly_pipeline.disassembly_cycle import Disassembly_cycle
from disassembly_pipeline.cell_init_utils import CellManager
from disassembly_pipeline.robot_init_utils import init_robot

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.grippers import VariableStiffnessGripper, SofthandGripper


class Pick_up_battery(EventState):

    '''
    Instantiates a robotblockset_python panda_ros object which can be used to control panda
    [...]

    ># robot_name string/string      The name of the robot

    #< robot_object []    The robot object which can be called upon

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #, input_keys = []

    def __init__(self, get_into_camera_position = True, safe_to_camera = True, pick_up_battery = True):
        super(Pick_up_battery, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['cy', 'dummy'])

        self.get_into_camera_position = get_into_camera_position
        self.safe_to_camera = safe_to_camera
        self.pick_up_battery = pick_up_battery
                    
    def on_enter(self, userdata):

        Logger.loginfo("Entering state Pick_up_battery")
        
        dummy = userdata.dummy
        if dummy:
            return 'continue'
        else:
            cy = userdata.cy
            cy.p2_pick_up_battery(get_into_camera_position = self.get_into_camera_position, safe_to_camera = self.safe_to_camera, pick_up_battery = self.pick_up_battery)

            return 'continue'

    def execute(self, userdata):
        
       
        
        return 'continue'
        


    def on_exit(self, userdata):

        return 'continue'
        


if __name__ == '__main__':

    print("Testing standalone")

    class userdata():

        def __init__(self):
            0
            
            #self.joints_data=pos

    
    rospy.init_node('test_node')
    #test_state=Instantiate_robotblockset()
    #test_state=CallJointTrap(0.5,0.1,)
    #usertest=userdata()
    #test_state.on_enter(usertest)
    #test_state.execute(usertest)
    #test_state.on_exit(usertest)

    #j=0.6
    #usertest=userdata([j,j,j,j,j,j,j])
    #test_state.on_enter(usertest)
    #test_state.execute(usertest)
    #test_state.on_exit(usertest)
