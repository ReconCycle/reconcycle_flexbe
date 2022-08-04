#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

#from sensor_msgs.msg import JointState

import time
#from robot_module_msgs.msg import JointTrapVelAction,JointTrapVelGoal
from robotblockset_python.panda_ros import panda_ros


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

    def __init__(self, robot_namespace):
        super(Instantiate_robotblockset, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['panda_1', 'panda_2'])

        # replace 'panda_2/joint_trap_vel_action_server' from dummies with 'joint_trap_vel_action_server'
        self._robot_namespace= robot_namespace
        
        self.robot = panda_ros(name = self._robot_namespace, init_node = False)

        
        
        #self._topic = self._namespace + '/joint_impedance_controller/move_joint_trap'
        #self._topic = '/joint_impedance_controller/move_joint_trap'

        #self._client = ProxyActionClient({ self._topic: JointTrapVelAction})
        #self._client = ProxyActionClient({self._topic: robot_module_msgs.msg.JointTrapVelAction}) # pass required clients as dict (topic: type)
        # JointTrapVelAction

		# It may happen that the action client fails to send the action goal.
            
    def on_enter(self, userdata):

        Logger.loginfo("Enter in state instantiate Robotblockset_python")
        
        # userdata.panda_1 = self.robot
        setattr(userdata, self._robot_namespace, self.robot)
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
    test_state=Instantiate_robotblockset(robot_namespace = 'panda_1')
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
