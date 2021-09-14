#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from sensor_msgs.msg import JointState

import time
from robot_module_msgs.msg import CartLinTaskAction,CartLinTaskGoal

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

class CallCartRel(EventState):

    '''
    [OBSOLETE] Calls cartesian relative server @ 'cart_rel' topic
    [...]

    ># joints_data string/int      The name under which the name is written into MongoDB

    #< tran_values []    The data read from mongoDB from specific _id (entry_name)

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self,exe_time,namespace=''):
        super(CallCartRel, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['trans_value'], output_keys = ['success'])

        # replace 'panda_2/joint_trap_vel_action_server' from dummies with 'joint_trap_vel_action_server'
        self._namespace=namespace
        
        self._topic = self._namespace + '/cart_rel'
        #self._topic = '/joint_impedance_controller/move_joint_trap'

        self._client = ProxyActionClient({ self._topic: CartLinTaskAction})
        
        self.exe_time=exe_time
		# It may happen that the action client fails to send the action goal.

            
    def on_enter(self, userdata):

        Logger.loginfo("Enter in state...")
        # JointState() is used for tests. Replace JointState() with userdata.joints_data.
        self.rel_move = userdata.trans_value #JointState()
        
        relative_pose = PoseStamped(pose=Pose(position=Point(self.rel_move[0],self.rel_move[1],  self.rel_move[2]), orientation=Quaternion(0, 0, 0, 1)))


    
        goal=CartLinTaskGoal([relative_pose.pose],self.exe_time, None)
       
        Logger.loginfo("Starting sending goal to...")
        Logger.loginfo('\n{}'.format(str(self._topic)))
        try:
            self._client.send_goal(self._topic,goal)
            Logger.loginfo("Goal sent: {}".format(str(goal)))
        except Exception as e:
			# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True
            return 'failed'

        self.timeout = time.time()

    def execute(self, userdata):
        

        try:
          
            
            #result = self._client.get_result(self._topic) 
            if self._client.has_result(self._topic):
                result = self._client.get_result(self._topic) 
                Logger.loginfo("Result {}".format(result))
                return 'continue'

            else:


            #if result == None:
                
                feedback = self._client.get_feedback(self._topic)
                Logger.loginfo("{}".format(feedback))
                #time.sleep(0.5)
                # 12 secs timeout
                
                #if time.time()-self.timeout > 12:
                    #break

                
 
            

        except Exception as e:
            
            Logger.loginfo("No result or server is not active!")
            return 'failed'

         

    def on_exit(self, userdata):

        Logger.loginfo("Exit state...")

        if not self._client.get_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
        Logger.loginfo('Finished sending goal to JointTrapVelGoal.')
        return 'continue'
        


if __name__ == '__main__':

    print("Testing standalone")

    class userdata():

        def __init__(self,pos):

            self.trans_value=pos

    
    rospy.init_node('test_node')
    test_state=CallCartRel(4,namespace='/panda_1')
    #test_state=CallJointTrap(0.5,0.1,)
    
    usertest=userdata([0.1,0,0])
    test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)

    
    usertest=userdata([-0.1,0,0])
    test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)
