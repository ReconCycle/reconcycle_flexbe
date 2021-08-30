#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time


class MoveCustomRobot(EventState):

    '''
    Calls JointMinJerkAction server @ '/joint_min_jerk_action_server' topic
    [...]
    
    -- robot_selection  string      namespace for robot 
    ># goal_joint_pos   list []     Position data input [float, float, ...] of 7 joints
    #< minjerk_out      list []  	minjerk action server reply userdata output
    <= continue                 Written successfully
    <= failed                  	Failed
    '''

    def __init__(self, robot_selection):
        super(MoveCustomRobot, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['goal_joint_pos'], output_keys = ['minjerk_out'])
        
        # declaration for end of topic
        self._end_of_topic = "/gazebo_panda/effort_joint_position_controller/command"
        self._joint_state_topic = "/joint_states"

        merge = '/' + str(robot_selection)
        self._publish_to_topic =  merge + self._end_of_topic
        self._get_from_topic = merge + self._joint_state_topic
        # rospy publisher
        self._publisher = rospy.Publisher(self._publish_to_topic, Float64MultiArray, queue_size=1)
        
        self.checker = True

    def execute(self, userdata):
        try:
            self._publisher.publish(self._data)
            self.goals = userdata.goal_joint_pos
            self.subscribe_to = rospy.Subscriber(self._get_from_topic, JointState, self.callback)
            
        except rospy.ROSInterruptException as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to publish data, check if subscriber is running... \n{}'.format(str(e)))

        return 'continue'
            
    def on_enter(self, userdata):
        # input [float, float, ...] of 7 joints is used for userdata.goal_joint_pos.
        self._data = Float64MultiArray()
        self._data.data = userdata.goal_joint_pos 

        Logger.loginfo("Publishing destination...")

        #return 'continue'

        
    def on_exit(self, userdata):
        Logger.loginfo('Finished publishing positions...')
        #return 'continue'

    def callback(self, msg):
        if str(msg.position[2])[0] == '-':
            #Logger.loginfo("Negative = {}".format(str(msg.position[2])[0:5]))
            if str(msg.position[2])[0:5] == str(self.goals[0]):
                self.subscribe_to.unregister()
                self.checker = False
                Logger.loginfo("Reached destination!")
        else:
            #Logger.loginfo("Positive = {}".format(float(str(msg.position[2])[0:4])))
            if str(msg.position[2])[0:4] == str(self.goals[0]):
                self.subscribe_to.unregister()
                self.checker = False
                Logger.loginfo("Reached destination!")