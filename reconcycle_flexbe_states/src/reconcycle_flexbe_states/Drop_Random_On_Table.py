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
import quaternion as qt

class DropRandomOnTable(EventState):
    
    '''
    A state that drops the object randomly on the table.
    [...]       

    -- namespace    string              namespace of publish topic  

    <= continue                         Written successfully
    <= failed                           Failed
    '''

    def __init__(self, namespace, exe_time):
        super(DropRandomOnTable, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['t2_out'])
        
        rospy.loginfo('__init__ callback happened.')   

        self.exe_time = exe_time
        self.ns = namespace
        
        # Define limits of XYZ and rotation
        self.low_xyz = [-0.05, -0.626, 0.08]
        self.high_xyz = [0.122, -0.581, 0.081]
        self.rot_limits = [-np.pi/3, np.pi/3]
        self.init_eulers = np.deg2rad([-124.62, 176.05, 143.00])
        self.H = np.zeros((4, 4))
        self.R = np.eye(4)
        
        # Declaring topic and client
        self._topic = self.ns + '/cartesian_impedance_controller/cart_lin_as'
        self._client = actionlib.SimpleActionClient(self._topic, robot_module_msgs.msg.CartLinTaskAction)
        
        self.result = ''

    def on_enter(self, userdata):
        Logger.loginfo("Started sending goal...")
        # create goal

        # orientation = self.init_eulers + [random_rot, 0, 0]
        # orientation = qt.from_euler_angles(orientation)
        # orientation = qt.as_float_array(orientation)[[1,2,3,0]]

        position = np.random.uniform(self.low_xyz, self.high_xyz)
        random_rot = np.random.uniform(self.rot_limits[0], self.rot_limits[1])
        orientation = (qt.from_euler_angles(self.init_eulers) * 
            qt.from_euler_angles([0, 0, random_rot]))
        # self.H[:3, :3] = qt.as_rotation_matrix(orientation)
        # self.H[:3, -1] = position
        # self.H[-1, -1] = 1.0
        # self.H = self.H.dot(self.R)
        # position = self.H[:3, -1]
        # orientation = qt.as_float_array(qt.from_rotation_matrix(self.H))[[1,2,3,0]]
        orientation = qt.as_float_array(orientation)[[1,2,3,0]]
        
        goal = robot_module_msgs.msg.CartLinTaskGoal() 
        pose = geometry_msgs.msg.Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        
        goal.target_pose = [pose]
        goal.desired_travel_time=self.exe_time 

        Logger.loginfo("Goal created: {}".format(goal))
        userdata.t2_out = [pose]
            
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
        #userdata.t2_out = self.result      
    
        return 'continue'        

    def on_exit(self, userdata):
        Logger.loginfo('Exiting call (CartLin).')
        return 'continue'

if __name__ == '__main__':
     print("Testing standalone")
     rospy.init_node('test_node')
     test_state=CallActionTFCartLin("test",3)
