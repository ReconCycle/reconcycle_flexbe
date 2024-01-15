#!/usr/bin/python

from flexbe_core import EventState, Logger

class CMoveState(EventState):

    def __init__(self, robot_name, pose, duration):
        super(CMoveState, self).__init__(outcomes = ['continue', 'failed'])

        self.robot_name = robot_name
        
        
        self.pose = pose
        self.duration = duration
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].CMove(
            self.pose,self.duration)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass