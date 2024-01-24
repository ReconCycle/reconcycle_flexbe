#!/usr/bin/python

from flexbe_core import EventState, Logger

class CPathState(EventState):

    def __init__(self, robot_name, path, duration):
        super(CPathState, self).__init__(outcomes = ['continue', 'failed'],
                                                    input_keys = ['robots'],
                                                    )

        self.robot_name = robot_name
        
        
        self.path = path
        self.duration = duration
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].CPath(
            self.path,self.duration)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass