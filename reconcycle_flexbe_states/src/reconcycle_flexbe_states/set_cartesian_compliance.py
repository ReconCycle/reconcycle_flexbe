#!/usr/bin/env python

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from robot_module_msgs.msg import ImpedanceParameters, CartesianCommand
from std_msgs.msg import Empty
from franka_msgs.msg import FrankaState
import numpy as np
import tf
import rospy

class SetCartesianComplianceProxyClient(EventState):

    '''
    Sets the compliance parameters of the IJS cartesian impedance controller.
    
    Parameters:   
    -- robot_name    string           Namespace of the robot 
    
    Run-time userdata:
    ># Kp            float[3]         stiffness for positional d.o.f
    ># Kr            float[3]         stiffness for rotational d.o.f
    
    Outcomes:
    <= continue
    <= failed
    '''
    
    def __init__(self, robot_name):
        super(SetCartesianComplianceProxyClient, self).__init__(input_keys=['Kp','Kr'], outcomes = ['continue', 'failed'])
        
        try:
            assert robot_name in ['panda_1', 'panda_2']
        except: 
            print("Robot name NOT in [panda_1, panda_2], FIX!")
            Logger.loginfo("SetCartesianCompliance INVALID ROBOT NAMESPACE")
            
        self.command_topic = "/" + str(robot_name) + "/cartesian_impedance_controller/command"
        self.reset_topic = "/" + str(robot_name) + "/cartesian_impedance_controller/reset_target"
        self.cart_stiff_topic =  "/" + str(robot_name) + "/cartesian_impedance_controller/stiffness"
        self.publisher = ProxyPublisher({self.command_topic: CartesianCommand, self.reset_topic: Empty, self.cart_stiff_topic: ImpedanceParameters}) 
        
        
        ## To track robot position, we need to listen to position messages.
        self.franka_state_topic = "/%s/franka_state_controller/franka_states"%robot_name
        self.subscriber = ProxySubscriberCached({self.franka_state_topic: FrankaState})
        
        self.outcome = 'failed' 
    
    def on_enter(self, userdata):
        # Check robot is in "CartesianImpedance" mode. If not: print("Not in cartesian impedance control strategy")
        
        holdPose = 'on' if not hasattr(userdata,'holdPose') else userdata.holdPose
        R = np.eye(3) if not hasattr(userdata,'R') else userdata.R
        D = 2.0 if not hasattr(userdata,'D') else userdata.D
        Kp = userdata.Kp
        Kr = userdata.Kr
     
        # Check input params 
        try:
            assert len(Kp) == 3
            assert len(Kr) == 3
            for i in Kp : assert i >= 0
            for i in Kr : assert i >= 0
            
            assert R.shape == (3,3)
            larger_args = np.argwhere(D > 2)
            assert len(larger_args) == 0
            smaller_args = np.argwhere(D<0)
            assert len(smaller_args) == 0
        except:
            # Handle exceptions
            print("SetCartesianCompliance INVALID input parameter (assertion check fail)")
            Logger.loginfo("SetCartesianCompliance INVALID input parameter (assertion check fail)")
            self.outcome = 'failed'
            return
            
        # Calculate stiff matrix
        trM = np.diag(Kp)
        rotM = np.diag(Kr)
        
        # Rotate
        trK = R * trM *np.transpose(R)
        rotK = rotM
        
        # Damping
        trD = R*2*np.sqrt(trM)*np.transpose(R)
        rotD = D*np.sqrt(rotM)
        
        #Check for NaN
        try:
            assert not np.isnan(trD).any()
            assert not np.isnan(rotD).any()
            assert not np.isnan(trK).any()
            assert not np.isnan(rotK).any()
        except:
            Logger.loginfo("SetCartesianCompliance NaNs present in trD/rotD/trK/rotK! (assertion check fail)")
            self.outcome = 'failed'
            return
            
        stiffness = ImpedanceParameters()
        stiffness.n = 9
        stiffness.k = np.concatenate((np.reshape(trK,(9,1)),np.reshape(rotK,(9,1))))
        stiffness.d = np.concatenate((np.reshape(trD,(9,1)),np.reshape(rotD,(9,1))))
        
        
        if holdPose == 'off':
            self.publisher.publish(self.cart_stiff_topic, stiffness)
        else:
            cmd_msg = CartesianCommand()
            cmd_msg.impedance = stiffness
            
            # Reset current robot target
            self.publisher.publish(self.reset_topic, Empty())
            
            # Get last robot joint positions
            if self.subscriber.has_msg(self.franka_state_topic):
                last_franka_state = self.subscriber.get_last_msg(self.franka_state_topic)
                self.subscriber.remove_last_msg(self.franka_state_topic)
            else:
                print('didnt get state')
                self.outcome = 'failed'
                return
            
            EE_matrix = np.reshape(last_franka_state.O_T_EE,(4,4),order='f')
            
            cmd_msg.pose.position.x = EE_matrix[0,3]
            cmd_msg.pose.position.y = EE_matrix[1,3]
            cmd_msg.pose.position.z = EE_matrix[2,3]
            quaternion = tf.transformations.quaternion_from_matrix(EE_matrix)    
            
            # tf library uses <x,y,z,w> quaternions
            cmd_msg.pose.orientation.x = quaternion[0]
            cmd_msg.pose.orientation.y = quaternion[1]
            cmd_msg.pose.orientation.z = quaternion[2]
            cmd_msg.pose.orientation.w = quaternion[3]
            
            cmd_msg.time_from_start = rospy.Duration(0,0)
            
            
            # Send command message
            self.publisher.publish(self.command_topic, cmd_msg)
        
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
    
    
    

