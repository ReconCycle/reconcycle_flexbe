#!/usr/bin/python

from flexbe_core import EventState, Logger

class CMoveForState(EventState):

    def __init__(self, robot_name, dx, t):
        super(CMoveForState, self).__init__(
            outcomes = ['continue', 'failed'],
            input_keys = ['robots'],
            )

        self.robot_name = robot_name
        
        
        self.dx = dx
        self.t = t
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].CMoveFor(
            dx = self.dx,t = self.t)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass