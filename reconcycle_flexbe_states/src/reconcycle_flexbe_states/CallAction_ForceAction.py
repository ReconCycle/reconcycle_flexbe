#!/usr/bin/python


import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from geometry_msgs.msg import Point

import time
from robot_module_msgs.msg import ForceActionAction, ForceActionGoal



class CallForceAction(EventState):

    '''
    Calls ForceAction server

    direction     [x, y, z]    # direction of sinusoidal motion pattern
    amplitude     float        # amplitude of the motion pattern 
    frequency     float        # frequency of the motion pattern
    force         float        # magnitude of the desired force
    rho_min       float        # this sets how soft the force is discarded

    bool                 success    # result of action

    <= continue                     Written successfully
    <= failed                       Failed
    '''

    def __init__(self, direction, amplitude, frequency, 
                 force, rho_min, namespace=''):
        super(CallForceAction, self).__init__(outcomes = ['continue', 'failed'], 
                                              output_keys = ['success'])

        self._namespace = namespace
        self._topic = (self._namespace +
            '/cartesian_impedance_controller_tum/force_action')
        self._client = ProxyActionClient({ self._topic: ForceActionAction})

        self.direction = direction
        self.amplitude = amplitude
        self.frequency = frequency
        self.force = force
        self.rho_min = rho_min
            
    def on_enter(self, userdata):
        Logger.loginfo("Entering Force Action state")
        
        point_direction = Point()
        point_direction.x = self.direction[0]
        point_direction.y = self.direction[1]
        point_direction.z = self.direction[2]

        force_goal = ForceActionGoal(point_direction, 
                               self.amplitude, 
                               self.frequency,
                               self.force,
                               self.rho_min)
        
        Logger.loginfo("Sending force goal")

        try:
            self._client.send_goal(self._topic, force_goal)
            Logger.loginfo("Goal sent: {}".format(str(force_goal)))
        except Exception as e:
            Logger.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True
            return 'failed'
        
        # self.timeout = time.time()

    def execute(self, userdata):
        
        # if time.time() - self.timeout > 10:
        #     return 'failed'

        try:
            if self._client.has_result(self._topic):
                result = self._client.get_result(self._topic) 
                Logger.loginfo("Result {}".format(result))
                return 'continue'
            else:
                feedback = self._client.get_feedback(self._topic)
                # Logger.loginfo("{}".format(feedback))
                
        except Exception as e:
            Logger.loginfo("No result or server is not active!")
            return 'failed'

    def on_exit(self, userdata):

        if not self._client.get_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Cancelled active action goal.')
            
        Logger.loginfo('Finished sending goal to ForceActionGoal.')
        return 'continue'
        


if __name__ == '__main__':

    print("Testing standalone")

    direction = [0, 1, 0]
    amplitude = 0.02
    frequency = 0.3
    force = 15.0
    rho_min = 0.7
    namespace = ''
    
    userdata = []
            
    rospy.init_node('test_force_action_node')
    
    test_state = CallForceAction(direction, amplitude, frequency, 
                                 force, rho_min, namespace)
    test_state.on_enter(userdata)
    test_state.execute(userdata)
    test_state.on_exit(userdata)

