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


class Pick_up_HCA(EventState):

    '''
    Instantiates a robotblockset_python panda_ros object which can be used to control panda
    [...]

    ># robot_name string/string      The name of the robot

    #< robot_object []    The robot object which can be called upon

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #, input_keys = []

    def __init__(self, init_safe=True, double_tap_vise=False, return_to_via= True):
        super(Pick_up_HCA, self).__init__(outcomes = ['kalo', 'qundis-pin', 'qundis-no_pin', 'failed'], input_keys = ['cy', 'dummy'], output_keys = ['hca_type', 'has_pin'])

        self.init_safe = init_safe
        self.double_tap_vise = double_tap_vise
        self.return_to_via = return_to_via
            
    def on_enter(self, userdata):

        Logger.loginfo("Entering state Pick_up_HCA")
        
        dummy = userdata.dummy
        
        if dummy:
            return 'qundis'
    
        else:
        
            cy = userdata.cy
            
            hca_type, has_pin = cy.p1_pickup_hca_and_put_into_vice(init_safe = self.init_safe, double_tap_vise = self.double_tap_vise, return_to_via = self.return_to_via)
            
            userdata.hca_type = hca_type
            userdata.has_pin = has_pin

            assert hca_type in ['qundis', 'kalo']

            if hca_type == 'kalo': outcome = 'kalo'
            if (hca_type == 'qundis') and has_pin: outcome = 'qundis-pin'
            else: outcome = 'qundis-no_pin'
            
            Logger.loginfo("Outcome: {}".format(outcome))
            self.outcome = outcome
            return outcome

    def execute(self, userdata):
        return self.outcome

    def on_exit(self, userdata):
        return self.outcome
        


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
    ##test_state.on_enter(usertest)
    #test_state.execute(usertest)
    #test_state.on_exit(usertest)
