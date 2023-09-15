#!/usr/bin/env python


from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyServiceCaller

import rospy
from flexbe_core import EventState, Logger
import time
from digital_interface_msgs.msg import DigitalState
from digital_interface_msgs.srv import PinStateWrite,PinStateWriteRequest

#for deblocking parallel execution in flexbe
import threading

class ActivateRaspiDigitalOutput(EventState):

    '''
    Calls service conected to raspberry GPIO
    [...]

    -- service_name  string		Speed data

    ># Digital_IO_value   bool	desired value
    #< success      bool  	check execution
    <= continue                 Written successfully
    <= failed                  	Failed
    '''

    def __init__(self, service_name):
        super(ActivateRaspiDigitalOutput, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['value'], output_keys = ['success'])

        self._service_name = service_name
        self._request = PinStateWriteRequest()
        
        self._pub=ProxyPublisher({service_name: DigitalState})
        self._srv=ProxyServiceCaller({service_name: PinStateWrite}, wait_duration = 0.5)
        
    def call_srv(self):

        self.response = self._srv.call(self._service_name,self.state)

    def execute(self, userdata):
 

        return 'continue'
            
    def on_enter(self, userdata):
        
        state = self._request

        state.value = userdata.value
     
        self.state=state

        #Logger.loginfo("Set digital Output...")

        try:
            Logger.loginfo("Goal sent: {}, {}".format(str(userdata.value), self._service_name))
            threading.Thread(target=self.call_srv).start()
            #self._srv.call(self._service_name,state)
     

        except Exception as e:
            print(e)
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('{} failed to set digital output:\n{}'.format(self._service_name, (e)))
            self._error = True

    def on_exit(self, userdata):

        return 'continue'

if __name__ == '__main__':

    print("Testing standalone")

    class userdata():

        def __init__(self,value):

            self.value=value

    rospy.init_node('test_node')
    test_state= ActivateRaspiDigitalOuput('/obr_activate')
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
