#!/usr/bin/python

from flexbe_core import EventState, Logger

class JPathState(EventState):

    def __init__(self, robot_name, path, duration):
        super(JPathState, self).__init__(outcomes = ['continue', 'failed'])

        self.robot_name = robot_name
        
        
        self.path = path
        self.duration = duration
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].JPath(
            self.path,self.duration)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass