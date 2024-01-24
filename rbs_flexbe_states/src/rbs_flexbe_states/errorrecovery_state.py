#!/usr/bin/python

from flexbe_core import EventState, Logger

class ErrorRecoveryState(EventState):

    def __init__(self, robot_name, ):
        super(ErrorRecoveryState, self).__init__(outcomes = ['continue', 'failed'],
                                                    input_keys = ['robots'],
                                                    )

        self.robot_name = robot_name
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].error_recovery()
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass