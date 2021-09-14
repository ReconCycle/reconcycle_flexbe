#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from sensor_msgs.msg import JointState

import time
from robot_module_msgs.msg import JointTrapVelAction,JointTrapVelGoal



class CallJointTrap(EventState):

    '''
    Calls JointTrapVelAction server

    ># joints_data string/int      The name under which the name is written into MongoDB

    #< joint_values []    The data read from mongoDB from specific _id (entry_name)

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self,max_vel, max_acl, namespace=''):
        super(CallJointTrap, self).__init__(outcomes = ['continue', 'failed'], 
                                            input_keys = ['joints_data'], 
                                            output_keys = ['joint_values'])

        self._namespace = namespace
        self._topic = self._namespace + '/joint_impedance_controller/move_joint_trap'
        self._client = ProxyActionClient({ self._topic: JointTrapVelAction})

        self.max_vel = max_vel
        self.max_acl = max_acl
            
    def on_enter(self, userdata):

        Logger.loginfo("Entering state...")
        self.goal_joint = userdata.joints_data
        joint = JointState()
        joint.position = self.goal_joint
        goal = JointTrapVelGoal([joint], self.max_vel, self.max_acl)

        
        Logger.loginfo("Starting sending goal...")

        try:
            self._client.send_goal(self._topic,goal)
            Logger.loginfo("Goal sent: {}".format(str(self.goal_joint)))
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

    class userdata():

        def __init__(self,pos):

            self.joints_data=pos

    
    rospy.init_node('test_node')
    test_state=CallJointTrap(0.5,0.1,namespace='')
    #test_state=CallJointTrap(0.5,0.1,)
    j=0.2
    usertest=userdata([j,j,j,j,j,j,j])
    #test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)

    j=0.6
    usertest=userdata([j,j,j,j,j,j,j])
    #test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)
