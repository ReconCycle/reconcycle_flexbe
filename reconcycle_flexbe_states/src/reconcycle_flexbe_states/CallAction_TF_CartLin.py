#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
from flexbe_core import EventState, Logger
import robot_module_msgs.msg
import actionlib


class CallActionTFCartLin(EventState):
    
    '''
    Implements a state that reads Pose() from input and sends them to ns/cart_lin_action_server
    [...]       

    #< t2_out       Data                Send TF data
    ># t2_data      Pose()              Data from target Pose() from mongodb
    -- namespace    string              namespace of publish topic  

    <= continue                         Written successfully
    <= failed                           Failed
    '''

    def __init__(self, namespace):
        rospy.loginfo('__init__ callback happened.')   
        super(CallActionTFCartLin, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['t2_data'], output_keys = ['t2_out'])
        
        self.ns = namespace
        # Declaring topic and client
        self._topic = self.ns + '/cart_lin_action_server'
        self._client = actionlib.SimpleActionClient(self._topic, robot_module_msgs.msg.CartLinTaskAction)

    def on_enter(self, userdata):
        Logger.loginfo("Started sending goal...")
        
        # create goal
        goal = robot_module_msgs.msg.CartLinTaskGoal([userdata.t2_data], 1, None)  
        Logger.loginfo("Goal created: {}".format(goal))
        try:
            # send goal and wait for result
            self._client.send_goal(goal)
            Logger.loginfo("Goal sent: {}".format(str(userdata.t2_data)))
            self._client.wait_for_result()
            
        except Exception as e:
            Logger.loginfo("No result or server is not active!")
            return 'failed'        
        
        # result from server
        self.result = self._client.get_result()
        Logger.loginfo("Server reply: \n {}".format(str(self.result)))    
        
    
    def execute(self, userdata):
        userdata.t2_out = self.result      
    
        return 'continue'        

    def on_exit(self, userdata):
        Logger.loginfo('Exiting call (CartLin).')
        return 'continue'
