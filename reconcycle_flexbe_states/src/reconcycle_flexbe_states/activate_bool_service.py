#!/usr/bin/env python


from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyServiceCaller

import rospy
from flexbe_core import EventState, Logger
import time
from std_srvs.srv import SetBool, SetBoolRequest
#for deblocking parallel execution in flexbe
import threading 

class ActivateBoolService(EventState):

    '''
    Calls an arbitrary service which uses std_msgs/Bool
    [...]
    
    -- service_name  string		Service name

    #< success      bool  	check execution
    <= continue                 Written successfully
    <= failed                  	Failed
    '''

    def __init__(self, service_name):
        super(ActivateBoolService, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['value'], output_keys = ['success'])

        
        self._srv=ProxyServiceCaller({service_name: SetBool}, wait_duration = 1)
        
        #self._client = ProxyActionClient({self._topic: robot_module_msgs.msg.JointMinJerkAction}) # pass required clients as dict (topic: type)
     
        self._service_name = service_name

    def call_srv(self):

        self.response = self._srv.call(self._service_name,self.state)

    def execute(self, userdata):
 
        return 'continue'
            
    def on_enter(self, userdata):
        
        state = SetBoolRequest()

        state.data = userdata.value
     
        self.state=state

        Logger.loginfo("Calling service {0}".format(self._service_name))

        try:
            #Logger.loginfo("Goal sent: {}".format(str(userdata.goal_joint_pos)))
            #threading.Thread(target=self.call_srv).start()
            output = self._srv.call(self._service_name,state)
            rospy.loginfo("Calling {0}: return {1}".format(self._service_name, output))
        except Exception as e:
            print(e)
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to call service:\n{}'.format(self._service_name))
            self._error = True

    def on_exit(self, userdata):

        return 'continue'





if __name__ == '__main__':

    print("Testing standalone")

    class userdata():

        def __init__(self,value):

            self.data=value


    rospy.init_node('test_node')
    test_state= ActivateBoolService('/vision_pipeline/enable')
    print('sleep')
    rospy.sleep(2)
    print('sleep')
    user_test=userdata(False)
    #test_state.on_enter(user_test)
    test_state.execute(user_test)
    test_state.on_exit(user_test)
    rospy.sleep(2)
    user_test=userdata(True)
    #test_state.on_enter(user_test)
    test_state.execute(user_test)
    test_state.on_exit(user_test)

    #rospy.spin()
