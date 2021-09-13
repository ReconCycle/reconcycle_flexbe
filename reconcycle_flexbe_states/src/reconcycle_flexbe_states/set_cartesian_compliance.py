#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from robot_module_msgs.msg import ImpedanceParameters, CartesianCommand
from std_msgs.msg import Empty
from franka_msgs.msg import FrankaState
import numpy as np
import tf

class SetCartesianComplianceProxyClient(EventState):

    '''
    FlexBE state that sets the cartesian impedance parameters
    
    -- robot_name    string                   Namespace of the robot 
    ># userdata      ImpedanceParameters      number of joints, stiffness and damping 
    
    <= continue
    <= failed
    '''
    
    def __init__(self, robot_name):
        super(SetCartesianComplianceProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        try:
            assert robot_name in ['panda_1', 'panda_2']
        except: 
            print("Robot name NOT in [panda_1, panda_2], FIX !")
            Logger.loginfo("SetCartesianCompliance INVALID ROBOT NAMESPACE")
            
        self.topic = "/" + str(robot_name) + "/cartesian_impedance_controller/command"
        self.client_pub = ProxyPublisher({self.topic: CartesianCommand})
        
        self.cart_stiff_topic =  "/" + str(robot_name) + "/cartesian_impedance_controller/stiffness"
        self.cart_stiff_pub = ProxyPublisher({self.cart_stiff_topic: ImpedanceParameters})
        
        # To reset target robot position:
        # send std_msgs/Empty msg to the specified topic. '/cartesian_impedance_controller/reset_target', 'std_msgs/Empty'
        self.reset_topic = "/" + str(robot_name) + "/cartesian_impedance_controller/reset_target"
        self.client_reset_pub = ProxyPublisher({self.reset_topic: Empty})
        
        ## To track robot position, , we need to listen to position messages.
        self.franka_state_topic = "/%s/franka_state"%robot_name
        self.state_subscriber = ProxySubscriberCached({self.franka_state_topic: FrankaState})
        # FrankaState:
        # http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html

    
    def on_enter(self, userdata):
        # Check robot is in "CartesianImpedance" mode. If not: print("Not in cartesian impedance control strategy")
     
        # Check input params 
        try:
            assert len(userdata.Kp) == 3
            assert len(userdata.Kr) == 3
            for i in userdata.Kp : assert i >= 0
            for i in userdata.Kr : assert i >= 0
            
            # If R and D are not specified, use current values of robot
            if userdata.R == None: 
                0 # Add
            if userdata.D == None:
                0 # Add
            
            assert userdata.R.shape == (3,3)
            larger_args = np.argwhere(userdata.D > 2)
            assert len(larger_args) == 0
            smaller_args = np.argwhere(userdata.D<0)
            assert len(smaller_args) == 0
        except:
            # Handle exceptions
            print("SetCartesianCompliance INVALID input parameter (assertion check fail)")
            Logger.loginfo("SetCartesianCompliance INVALID input parameter (assertion check fail)")
            self.outcome = 'failed'
            return
            
        # Calculate stiff matrix
        trM = np.diag(userdata.Kp)
        rotM = np.diag(userdata.Kr)
        
        # Rotate
        trK = userdata.R * trM *np.transpose(userdata.R)
        rotK = rotM
        
        # Damping
        trD = userdata.R*2*np.sqrt(trM)*np.transpose(userdata.R)
        rotD = userdata.D*np.sqrt(rotM)
        
        #Check for NaN
        try:
            assert not np.isnan(trD).any()
            assert not np.isnan(rotD).any()
            assert not np.isnan(trK).any()
            assert not np.isnan(rotK).any()
        except:
            Logger.loginfo("SetCartesianCompliance NaNs present in trD/rotD/trK/rotK !(assertion check fail)")
            self.outcome = 'failed'
            return
            
        stiffness = ImpedanceParameters()
        stiffness.n = 9
        stiffness.k = [np.reshape(tdK, (9,1)), np.reshape(rotK, (9,1))]
        stiffness.d = [np.reshape(trD, (9,1)), np.reshape(rotD, (9,1))]
        
        if userdata.holdPose == 'off':
            self.cart_stiff_pub.publish(self.cart_stiff_topic, stiffness)
        
        else:
            cmd_msg = CartesianCommand()
            cmd_msg.Impedance = stiffness
            
            # Reset current robot target
            self.client_reset_pub.publish(self.reset_topic, Empty())
            
            # Get last robot joint positions
            last_franka_state = self.state_subscriber.get_last_msg(self.franka_state_topic)
            
            EE_matrix = last_franka_state.O_T_EE
            
            cmd_msg.pose.position.X = robot.EE_matrix[12]
            cmd_msg.pose.position.Y = robot.EE_matrix[13]
            cmd_msg.pose.position.Z = robot.EE_matrix[14]
                        
            # EE_matrix is COLUMN_MAJOR !
            rotation_matrix = np.array([[EE_matrix[0], EE_matrix[4], EE_matrix[8]],
                                        [EE_matrix[1], EE_matrix[5], EE_matrix[9]],
                                        [EE_matrix[2], EE_matrix[6], EE_matrix[10]]], dtype= np.double)
                                        
            quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)     # PREVERI CE JE SPLOH PRAV !!!!!!!
            
            cmd_msg.pose.orientation.W = quaternion[0]
            cmd_msg.pose.orientation.X = quaternion[1]
            cmd_msg.pose.orientation.Y = quaternion[2]
            cmd_msg.pose.orientation.Z = quaternion[3]
            
            # Send command message
            self.client_pub.publish(self.topic, cmd_msg)
        
        Logger.loginfo("Setting CartesianCompliance...")        
        self.outcome = 'continue'
        
    def execute(self, userdata):
        return self.outcome

    def on_exit(self, userdata):
        Logger.loginfo("Exiting SetCartesianCompliance!")
    
    def on_start(self):
        pass

    def on_stop(self):
        pass