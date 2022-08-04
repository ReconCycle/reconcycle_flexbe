#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from robot_module_msgs.msg import CartLinTaskAction, CartLinTaskGoal
import tf.transformations as tft
import numpy as np
import quaternion as qt

class RBS_dummy_CMove_data(EventState):
    
    '''
    Implements a state that reads Pose() from input and sends them to ns/cart_lin_action_server
    [...]       

    #< t2_out       Data                Send TF data
    ># t2_data      Pose()              Data from target Pose() from mongodb
    -- namespace    string              namespace of publish topic  

    <= continue                         Written successfully
    <= failed                           Failed
    '''

    def __init__(self, cart_pos, RPY, t):
        super(RBS_dummy_CMove_data, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['t2_data'])
        
        rospy.loginfo('__init__ callback happened.')   
        

    def on_enter(self, userdata):
        
        #position = np.array([userdata.t2_data[0].position.x,
        #                     userdata.t2_data[0].position.y,
        #                     userdata.t2_data[0].position.z])
        #orientation = np.array([userdata.t2_data[0].orientation.x,
        #                        userdata.t2_data[0].orientation.y,
        #                        userdata.t2_data[0].orientation.z,
        #                        userdata.t2_data[0].orientation.w])
        userdata.t2_data = 0
    
    def execute(self, userdata):
        0
        return 'continue'
       

if __name__ == '__main__':
     print("Testing standalone")
     rospy.init_node('test_node')
     test_state=RBS_dummy_CMove_data("test", 3)
