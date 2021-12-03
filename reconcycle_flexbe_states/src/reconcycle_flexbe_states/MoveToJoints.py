#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from sensor_msgs.msg import JointState

import time
from robot_module_msgs.msg import JointTrapVelAction,JointTrapVelGoal



class MoveToJoints(EventState):

    '''
    Moves the robot to specified joint positions

    parameters:
    joints          float array
    max_vel         float
    max_acl         float

    #< joint_values []    The specified joints

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self, joints, max_vel, max_acl, namespace=''):
        super(MoveToJoints, self).__init__(outcomes = ['continue', 'failed'], 
                                           output_keys = ['joint_values'])

        self._namespace = namespace
        self._topic = self._namespace + '/joint_impedance_controller/move_joint_trap'
        self._client = ProxyActionClient({ self._topic: JointTrapVelAction})

        self.joints = joints
        self.max_vel = max_vel
        self.max_acl = max_acl
            
    def on_enter(self, userdata):

        Logger.loginfo("Entering state...")
        joint_state = JointState()
        joint_state.position = self.joints
        goal = JointTrapVelGoal([joint_state], self.max_vel, self.max_acl)

        
        Logger.loginfo("Starting sending goal...")

        try:
            self._client.send_goal(self._topic, goal)
            Logger.loginfo("Goal sent: {}".format(str(self.joints)))
        except Exception as e:
            Logger.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True
            return 'failed'

        #self.timeout = time.time()

    def execute(self, userdata):
        
        try:
            
            #if time.time()-self.timeout > 12:
            #        break
                    
            if self._client.has_result(self._topic):
                result = self._client.get_result(self._topic) 
                #Logger.loginfo("Result {}".format(result))
                return 'continue'
            else:
                feedback = self._client.get_feedback(self._topic)
                #Logger.loginfo("{}".format(feedback))
                
        except Exception as e:
            Logger.loginfo("No result or server is not active!")
            return 'failed'

    def on_exit(self, userdata):

        if not self._client.get_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
            
        Logger.loginfo('Finished sending goal to JointTrapVelGoal.')
        return 'continue'
        


if __name__ == '__main__':

    print("Testing standalone")

    rospy.init_node('test_node')
