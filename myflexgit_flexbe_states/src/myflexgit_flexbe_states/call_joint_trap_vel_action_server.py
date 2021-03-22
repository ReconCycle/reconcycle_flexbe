#!/usr/bin/python3

import actionlib
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from sensor_msgs.msg import JointState


from robot_module_msgs.msg import JointTrapVelAction,JointTrapVelGoal



class CallJointTrap(EventState):

    '''
    Calls JointTrapVelAction server @ 'joint_trap_vel_action_server' topic
    [...]

    ># joints_data string/int      The name under which the name is written into MongoDB

    #< joint_values []    The data read from mongoDB from specific _id (entry_name)

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self,max_vel,max_acl,namespace=''):
        super(CallJointTrap, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['joints_data'], output_keys = ['joint_values'])

        # replace 'panda_2/joint_trap_vel_action_server' from dummies with 'joint_trap_vel_action_server'
        self._namespace=namespace
        #self._topic = self._namespace + '/joint_impedance_controller/move_joint_trap'
        self._topic =  '/joint_impedance_controller/move_joint_trap'

        self._client = ProxyActionClient({ self._topic: JointTrapVelAction})
        #self._client = ProxyActionClient({self._topic: robot_module_msgs.msg.JointTrapVelAction}) # pass required clients as dict (topic: type)
        # JointTrapVelAction

		# It may happen that the action client fails to send the action goal.

    def execute(self, userdata):
        

        try:
            result = self._client.get_result()
            userdata.joint_values = result 
            Logger.loginfo("Action Server reply: \n {}".format(str(userdata.joint_values)))
        except Exception as e:
            Logger.loginfo("No result or server is not active!")
            return 'failed'

        return 'continue'
            
    def on_enter(self, userdata):
        # JointState() is used for tests. Replace JointState() with userdata.joints_data.
        self.goal_joint = userdata.joints_data #JointState()
        joint = JointState()
        joint.position = self.goal_joint
        goal = JointTrapVelGoal([joint], 0.3, 0.3)

        print(goal)
        Logger.loginfo("Starting sending goal...")

        try:
            self._client.send_goal(self._topic,goal)
            Logger.loginfo("Goal sent: {}".format(str(self.goal_joint)))
        except Exception as e:
			# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True

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
    test_state=CallJointTrap(0.5,0.1)
    usertest=userdata([1,1,1,1,1,1,1])
    test_state.on_enter(usertest)
    #test_state.execute(usertest)
    #test_state.on_exit(usertest)


   # usertest=userdata([0.1,0.1,0.1,0.1,0.1,0.1,0.1])
    #test_state.on_enter(usertest)
    #test_state.execute(usertest)
    #test_state.on_exit(usertest)