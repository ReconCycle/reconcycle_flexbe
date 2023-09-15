#!/usr/bin/python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient


import time
import numpy as np
import sys
import rospy
import os

from disassembly_pipeline.multithreading import do_concurrently



class ActionPredictor_d(EventState):

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
        super(ActionPredictor_d, self).__init__(outcomes = ['hca_to_vise', 'pinpush', 'levering', 'pcb_to_cutter', 'cutting', 'pick_up_battery', 'hca_frame_to_bin', 'failed'])
        
        self.list_of_actions = ['hca_to_vise', 'pinpush', 'levering', 'pcb_to_cutter', 'cutting', 'pick_up_battery', 'hca_frame_to_bin']
        self.cur_n = 0

                    
    def on_enter(self, userdata):

        Logger.loginfo("Entering state ActionPredictor_d")
        
        cur_action = self.list_of_actions[self.cur_n]
        
        self.cur_n += 1
        if self.cur_n == len(self.list_of_actions): self.cur_n = 0
        
        return cur_action

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
