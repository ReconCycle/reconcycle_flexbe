#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose
#import tf2_geometry_msgs
from flexbe_core import EventState, Logger
import robot_module_msgs.msg
import geometry_msgs.msg
import actionlib
import tf.transformations as tft
import numpy as np


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

    def __init__(self, namespace, exe_time, offset=0):
        super(CallActionTFCartLin, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['t2_data'], output_keys = ['t2_out'])
        
        
        rospy.loginfo('__init__ callback happened.')   
        
        self.exe_time = exe_time
        self.ns = namespace
        if offset == 0:
            offset = [0, 0, 0, 0, 0, 0, 1]
        self.offset = np.array(offset)
        # Declaring topic and client
        self._topic = self.ns + '/cartesian_impedance_controller/cart_lin_as'
        self._client = actionlib.SimpleActionClient(self._topic, robot_module_msgs.msg.CartLinTaskAction)
        
        self.result = ''

    def on_enter(self, userdata):
        Logger.loginfo("Started sending goal...")
        # create goal

        position = np.array([userdata.t2_data[0].position.x,
                             userdata.t2_data[0].position.y,
                             userdata.t2_data[0].position.z])
        orientation = np.array([userdata.t2_data[0].orientation.x,
                                userdata.t2_data[0].orientation.y,
                                userdata.t2_data[0].orientation.z,
                                userdata.t2_data[0].orientation.w])
        # TODO: Implement offset calculation for position and orientation!
        position = position + self.offset[0:3]
        orientation = tft.quaternion_multiply(orientation, self.offset[3:])

        # M = tft.compose_matrix(translate=position, angles=tft.euler_from_quaternion(orientation))
        # R = tft.compose_matrix(translate=[0.2,0.0])
      

        #goal = robot_module_msgs.msg.CartLinTaskActionGoal() 
        #goal.goal.target_pose=[userdata.t2_data]
        #goal.goal.desired_travel_time=self.exe_time 
        goal = robot_module_msgs.msg.CartLinTaskGoal() 
        pose = geometry_msgs.msg.Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        
        goal.target_pose=[pose]
        
        print(goal.target_pose)

        #goal.target_pose=[userdata.t2_data]
        goal.desired_travel_time=self.exe_time 

        Logger.loginfo("Goal created: {}".format(goal))

            
        try:
            # send goal and wait for result
            self._client.send_goal(goal)
            Logger.loginfo("Goal sent: {}".format(str(goal)))
            self._client.wait_for_result()
            
        except Exception as e:
            print(e)
            Logger.loginfo("No result or server is not active!")
            self.result=self._client.get_result()
            Logger.loginfo("Server reply: \n {}".format(str(self.result)))  
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

if __name__ == '__main__':
     print("Testing standalone")
     rospy.init_node('test_node')
     test_state=CallActionTFCartLin("test",3)
