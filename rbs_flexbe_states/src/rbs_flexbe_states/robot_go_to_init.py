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

from disassembly_pipeline.multithreading import do_concurrently
class Robot_go_to_init(EventState):

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
        super(Robot_go_to_init, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['cy', 'p1', 'p2'])
    
    def on_enter(self, userdata):

        Logger.loginfo("Enter in state instantiate Robotblockset_python")
        cy = userdata.cy
        p1 = userdata.p1
        p2 = userdata.p2
        #robot = getattr(userdata, self.robot_name)
        
        #cy.robot_go_to_init_and_open_gripper(robot_obj = robot)

        do_concurrently([[cy.robot_go_to_init_and_open_gripper, {'robot_obj':p1}],
                        [cy.robot_go_to_init_and_open_gripper, {'robot_obj': p2}],
                        [cy.prepare_vision, {}],
                        [cy.prepare_pneumatics, {}]], wait_until_task_completion=True)

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
