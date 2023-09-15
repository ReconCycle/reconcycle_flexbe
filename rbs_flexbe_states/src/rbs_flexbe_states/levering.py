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


class Levering(EventState):

    '''
    Instantiates a robotblockset_python panda_ros object which can be used to control panda
    [...]

    ># robot_name string/string      The name of the robot

    #< robot_object []    The robot object which can be called upon

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #, input_keys = []

    def __init__(self):
        super(Levering, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['cy', 'hca_type', 'levering_direction_deg', 'adaptive_levering', 'dummy'], output_keys = ['adaptive_levering_dT_1' ,'adaptive_levering_dT_2'])

        0
    
            
    def on_enter(self, userdata):

        Logger.loginfo("Entering state instantiate Levering")
        
        dummy = userdata.dummy
        if dummy:
            return 'continue'
        else:
         
            cy = userdata.cy
            
            hca_type = userdata.hca_type
        
            direction_deg = userdata.levering_direction_deg
            
            adaptive = userdata.adaptive_levering
            
            cy.p2_perform_levering(hca_type = hca_type, direction_deg = direction_deg, adaptive = adaptive)
            
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
