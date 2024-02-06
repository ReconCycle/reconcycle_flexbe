#!/usr/bin/python

from flexbe_core import EventState, Logger

class CMoveState(EventState):

    def __init__(self, robot_name, x, t):
        super(CMoveState, self).__init__(
            outcomes = ['continue', 'failed'],
            input_keys = ['robots'],
            )

        self.robot_name = robot_name
        
        
        self.x = x
        self.t = t
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].CMove(
            x = self.x,t = self.t)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass