#!/usr/bin/python

from flexbe_core import EventState, Logger

class CNCCutState(EventState):

    def __init__(self, ):
        super(CNCCutState, self).__init__(outcomes = ['continue', 'failed'])
        
        
    def on_enter(self, userdata):
        self.x = userdata.x
        
        userdata.CNC.on_enter(
            self.x)

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass