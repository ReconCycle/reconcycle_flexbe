#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from robot_module_msgs.msg import CartLinTaskAction, CartLinTaskGoal
import tf.transformations as tft
import numpy as np
import quaternion as qt

class RBS_CMove(EventState):
    
    '''
    Implements a state that reads Pose() from input and sends them to ns/cart_lin_action_server
    [...]       

    #< t2_out       Data                Send TF data
    ># t2_data      Pose()              Data from target Pose() from mongodb
    -- namespace    string              namespace of publish topic  

    <= continue                         Written successfully
    <= failed                           Failed
    '''

    def __init__(self, robot_name, t, local_offset=0, global_pos_offset=0, limit_rotations=False):
        super(RBS_CMove, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['t2_data'], output_keys = ['t2_out'])
        
        rospy.loginfo('__init__ callback happened.')   
        
        self.exe_time = exe_time
        self.ns = namespace
        if local_offset == 0:
            local_offset = [0, 0, 0, 0, 0, 0]
        if global_pos_offset == 0:
            global_pos_offset = [0, 0, 0]
        self.local_offset = np.array(local_offset)
        self.global_pos_offset = np.array(global_pos_offset)
        self.limit_rotations = limit_rotations
        self.H = np.zeros((4, 4))
        self.H[-1, -1] = 1.0
        self.R = np.eye(4)
        # Declaring topic and client
        self._topic = self.ns + '/cartesian_impedance_controller/cart_lin_as'
        self._client = ProxyActionClient({self._topic: CartLinTaskAction})
        
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
        old_position = position
        old_orientation = orientation
        
        Logger.loginfo("Local offset: {}".format(self.local_offset))
        Logger.loginfo("Global offset: {}".format(self.global_pos_offset))
    
        ###### LOCAL OFFSET
        orientation = (qt.from_float_array(old_orientation[[3,0,1,2]]) * 
            qt.from_euler_angles(np.deg2rad(self.local_offset[3:])))
        self.R[:3, -1] = self.local_offset[:3]
        
        eulers = qt.as_euler_angles(orientation)
        Logger.loginfo("Local eulers: {}".format(eulers))

        self.H[:3, :3] = qt.as_rotation_matrix(orientation)
        self.H[:3, -1] = old_position
        self.H = self.H.dot(self.R)
        position = self.H[:3, -1]
        orientation = qt.as_float_array(qt.from_rotation_matrix(self.H))[[1,2,3,0]]

        eulers = qt.as_euler_angles(qt.from_float_array(orientation[[3,0,1,2]]))
        eulers = np.rad2deg(eulers)
        Logger.loginfo("Eulers are: {}".format(eulers)) 

        ###### GLOBAL OFFSET
        position = position + self.global_pos_offset

        # orientation = (qt.from_float_array(orientation[[3,0,1,2]]) * 
        #     qt.from_euler_angles(np.deg2rad(self.offset[3:])))
        # orientation = qt.as_float_array(orientation)[[1,2,3,0]]
    
        # # Fix rotation if required (NOT REQUIRED CURRENTLY)
        # if self.limit_rotations:
        #     if (eulers[2] < -30 or eulers[2] > 70):
        #         self.offset[-1] += 180
        #         orientation = (qt.from_float_array(old_orientation[[3,0,1,2]]) * 
        #             qt.from_euler_angles(np.deg2rad(self.offset[3:])))
        #         self.R[:3, -1] = self.offset[:3]
        #         self.H[:3, :3] = qt.as_rotation_matrix(orientation)
        #         self.H[:3, -1] = old_position
        #         self.H = self.H.dot(self.R)
        #         position = self.H[:3, -1]
        #         orientation = qt.as_float_array(qt.from_rotation_matrix(self.H))[[1,2,3,0]]
        #         eulers = qt.as_euler_angles(qt.from_float_array(orientation[[3,0,1,2]]))
        #         eulers = np.rad2deg(eulers)
        #         Logger.loginfo("Fixed eulers are: {}".format(eulers)) 
        #     else:
        #         Logger.loginfo("Eulers are fine!") 

        goal = CartLinTaskGoal() 
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        
        goal.target_pose = [pose]
        goal.desired_travel_time = self.exe_time 

        Logger.loginfo("Goal created: {}".format(goal))
            
        try:
            self._client.send_goal(self._topic, goal)
            Logger.loginfo("Goal sent: {}".format(str(goal)))
        except Exception as e:
            Logger.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True
            return 'failed'       
    
    def execute(self, userdata):
        try:
            if self._client.has_result(self._topic):
                self.result = self._client.get_result(self._topic) 
                return 'continue'
            else:
                feedback = self._client.get_feedback(self._topic)
        except Exception as e:
            Logger.loginfo("No result or server is not active!")
            return 'failed'

    def on_exit(self, userdata):
        userdata.t2_out = self.result

        if not self._client.get_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
            
        Logger.loginfo('Finished sending CartLinTaskGoal.')


if __name__ == '__main__':
     print("Testing standalone")
     rospy.init_node('test_node')
     test_state=CallActionTFCartLin("test", 3)
