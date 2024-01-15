#!/usr/bin/python

from flexbe_core import EventState, Logger

class JMoveState(EventState):

    def __init__(self, robot_name, q, duration):
        super(JMoveState, self).__init__(outcomes = ['continue', 'failed'])

        self.robot_name = robot_name
        
        
        self.q = q
        self.duration = duration
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].JMove(
            self.q,self.duration)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass