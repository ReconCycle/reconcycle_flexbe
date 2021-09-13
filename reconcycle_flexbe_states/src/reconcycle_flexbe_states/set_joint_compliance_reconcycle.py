#!/usr/bin/env python

import numpy as np
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from robot_module_msgs.msg import ImpedanceParameters, CartesianCommand, JointCommand
from franka_msgs.msg import FrankaState
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

class SetJointComplianceProxyClient(EventState):

    '''
    FlexBE state that sets the joint impedance parameters
    
    -- robot_name    string                   Namespace of the robot 
    ># userdata      ImpedanceParameters      number of joints, stiffness and damping 
    
    <= continue
    <= failed
    '''
    
    def __init__(self, robot_name):
        super(SetJointComplianceProxyClient, self).__init__(outcomes = ['continue', 'failed'])
        
        try:
            assert robot_name in ['panda_1', 'panda_2']
        except: 
            print("Robot name NOT in [panda_1, panda_2], FIX !")
            Logger.loginfo("SetJointCompliance INVALID ROBOT NAMESPACE")

        self.topic = "/" + str(robot_name) + "/joint_impedance_controller/command"
        self.client_pub = ProxyPublisher({self.topic: JointCommand})
        
        # To reset robot position:
        # send std_msgs/Empty msg to the specified topic. '/joint_impedance_controller/reset_target', 'std_msgs/Empty'
        self.reset_topic = "/" + str(robot_name) + "/joint_impedance_controller/reset_target"
        self.client_reset_pub = ProxyPublisher({self.reset_topic: Empty})
        
        ## To track robot position, , we need to listen to position messages.
        self.franka_state_topic = "/%s/franka_state"%robot_name
        self.state_subscriber = ProxySubscriberCached({self.franka_state_topic: FrankaState})
        # FrankaState:
        # http://docs.ros.org/en/api/sensor_msgs/html/msg/JointState.html
            
    def on_enter(self, userdata):
     
        # Check input params 
        try:
            assert len(userdata.k) == 7
            assert len(userdata.d) == 7
            for i in userdata.k : assert i >= 0
            for i in userdata.d : assert i >= 0
            assert not np.isnan(userdata.k).any()
            assert not np.isnan(userdata.d).any()
        except:
            # Handle exceptions
            print("SetJointCompliance INVALID input parameter (assertion check fail)")
            Logger.loginfo("SetJointCompliance INVALID input parameter (assertion check fail)")
            self.outcome = 'failed'
            return
            
        ###### 
        # Check robot is in Joint Impedance control strategy. if not: print("Not in joint impedance control strategy")
        
        Logger.loginfo("Setting JointCompliance...")        

        # Get last robot joint positions
        last_franka_state = self.state_subscriber.get_last_msg(self.franka_state_topic)

        q = last_franka_state.q
        
        cmd_msg = JointCommand() # Generate command message
        
        self.client_reset_pub.publish(self.reset_topic, Empty())
        
        cmd_msg.pos = q
        cmd_msg.vel = np.zeros((7,1))
        cmd_msg.trq = np.zeros((7,1))
        cmd_msg.impedance = userdata
        # Send the message, finally 
        self.client_pub.publish(self.topic, cmd_msg)
        self.outcome = 'continue'
        
    def execute(self, userdata):
        return self.outcome

    def on_exit(self, userdata):
        Logger.loginfo("Exiting SetJointCompliance!")
    
    def on_start(self):
        pass

    def on_stop(self):
        pass
