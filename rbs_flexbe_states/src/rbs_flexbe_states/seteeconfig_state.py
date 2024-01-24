#!/usr/bin/python

from flexbe_core import EventState, Logger

class SetEEConfigState(EventState):

    def __init__(self, robot_name, ee_config):
        super(SetEEConfigState, self).__init__(outcomes = ['continue', 'failed'],
                                                    input_keys = ['robots'],
                                                    )

        self.robot_name = robot_name
        
        
        self.ee_config = ee_config
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].SetNewEEConfig(
            self.ee_config)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass