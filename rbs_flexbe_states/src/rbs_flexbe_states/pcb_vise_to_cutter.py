#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

#from sensor_msgs.msg import JointState

import time
import numpy as np
import sys
import rospy
import warnings
warnings.filterwarnings('ignore')

from disassembly_pipeline.disassembly_cycle import Disassembly_cycle
from disassembly_pipeline.cell_init_utils import CellManager
from disassembly_pipeline.robot_init_utils import init_robot

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.grippers import VariableStiffnessGripper, SofthandGripper


class PCB_vise_to_cutter(EventState):

    '''
    Instantiates a robotblockset_python panda_ros object which can be used to control panda
    [...]

    ># robot_name string/string      The name of the robot

    #< robot_object []    The robot object which can be called upon

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #, input_keys = []

    def __init__(self, init_safe = True, activate_cutter = False, move_to_realsense_pos=True):
        super(PCB_vise_to_cutter, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['cy', 'hca_type', 'dummy'])

        self.init_safe = init_safe
        self.activate_cutter = activate_cutter
        self.move_to_realsense_pos = move_to_realsense_pos
            
    def on_enter(self, userdata):

        Logger.loginfo("Enter in state PCB_vise_to_cutter")
        dummy = userdata.dummy
        if dummy:
            return 'continue'
        else:
        
            cy = userdata.cy
            
            hca_type = userdata.hca_type
            
            cy.p2_handle_pcb_to_cutter(hca_type = hca_type, init_safe = self.init_safe, activate_cutter=self.activate_cutter, move_to_realsense_pos = self.move_to_realsense_pos)
            
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
    test_state=Instantiate_robotblockset()
    #test_state=CallJointTrap(0.5,0.1,)
    usertest=userdata()
    test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)

    j=0.6
    usertest=userdata([j,j,j,j,j,j,j])
    #test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)
