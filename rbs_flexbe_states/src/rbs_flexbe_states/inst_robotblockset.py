#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

#from sensor_msgs.msg import JointState

import time
import numpy as np

import rospy
import warnings
warnings.filterwarnings('ignore')

from disassembly_pipeline.disassembly_cycle import Disassembly_cycle
from disassembly_pipeline.cell_init_utils import CellManager
from disassembly_pipeline.robot_init_utils import init_robot
from disassembly_pipeline.multithreading import do_concurrently

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.grippers import VariableStiffnessGripper, SofthandGripper


class Instantiate_robotblockset(EventState):

    '''
    Instantiates a robotblockset_python panda_ros object which can be used to control panda
    [...]

    ># robot_name string/string      The name of the robot

    #< robot_object []    The robot object which can be called upon

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #, input_keys = []

    def __init__(self, cycle_speed):
        super(Instantiate_robotblockset, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['p1', 'p2', 'cy'])
        
        
        # replace 'panda_2/joint_trap_vel_action_server' from dummies with 'joint_trap_vel_action_server'
                
        # Panda 1 init
        p1 = init_robot('panda_1', start_controller = 'joint_impedance_controller', init_node = False)
        #p1.SetCollisionBehavior(F=50, T= 10, tq = 20)
        sh_grp = SofthandGripper()
        p1.gripper = sh_grp
        p1.gripper.open()
        p1.ResetCurrentTarget()
        #p1.JMove(p1_q1_init, t = 2, max_vel=0.5, max_acc= 0.5)
        #########
        
        # Panda 2 init
        #p2 = init_robot('panda_2', start_controller = 'joint_impedance_controller')
        p2 = init_robot('panda_2', start_controller = 'cartesian_impedance_controller', init_node = False)
        #p2.SetCollisionBehavior(F=50, T= 20, tq = 30) # Now in do concurrently

        sc_grp = VariableStiffnessGripper()
        p2.gripper = sc_grp
        p2.gripper.open()
        p2.ResetCurrentTarget()

        p2._verbose = 1
        p2.SetLoggerLevel(level='debug')
        
        do_concurrently([[p1.SetCollisionBehavior, {'F':50, 'T':10, 'tq':20}], [p2.SetCollisionBehavior, {'F':50, 'T':20, 'tq':30}]], wait_until_task_completion = True)
        
        
        self.p1 = p1
        self.p2 = p2

        # MAKE SURE panda_1 or panda_2 are initialized before calling this ! (they call init_node() which is required!)
        cellmanager = CellManager()

        j, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st, sidejaws_st, \
        slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st,tfread_st,  get_next_action_st= cellmanager.init_cell()
    
        # This will actually move the robots

        cy = Disassembly_cycle(p1, p2, ud, activate_basler_st, activate_realsense_st, activate_block2_st, mainjaws_st,
                            sidejaws_st, slider_st, clamptray_st, rotate_holder_st, cutter_st, cutter_airblower_st,
                            tfread_st, get_next_action_st)

        rospy.set_param("/vision/basler/publish_labeled_img", True)
        rospy.set_param("/vision/realsense/publish_labeled_img", True)
        rospy.set_param("/vision/realsense/publish_depth_img", False)
        rospy.set_param("/vision/realsense/publish_cluster_img", False)

        cy.prepare_pneumatics()
        cy.set_cycle_speed(cycle_speed)

        
        
        self.cy = cy
        
        #self.robot = panda_ros(name = self._robot_namespace, init_node = False)        
        #self._topic = self._namespace + '/joint_impedance_controller/move_joint_trap'
        #self._topic = '/joint_impedance_controller/move_joint_trap'

        #self._client = ProxyActionClient({ self._topic: JointTrapVelAction})
        #self._client = ProxyActionClient({self._topic: robot_module_msgs.msg.JointTrapVelAction}) # pass required clients as dict (topic: type)
        # JointTrapVelAction

		# It may happen that the action client fails to send the action goal.
            
    def on_enter(self, userdata):

        Logger.loginfo("Enter in state instantiate Robotblockset_python")
        
        # userdata.panda_1 = self.robot
        setattr(userdata, 'p1' , self.p1)
        setattr(userdata, 'p2' , self.p2)
        setattr(userdata, 'cy' , self.cy)

        #userdata.panda_n = self.robot

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
