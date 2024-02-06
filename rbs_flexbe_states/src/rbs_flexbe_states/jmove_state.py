#!/usr/bin/python

from flexbe_core import EventState, Logger

class JMoveState(EventState):

    def __init__(self, robot_name, q, t, qdot_max_factor, qddot_max_factor):
        super(JMoveState, self).__init__(
            outcomes = ['continue', 'failed'],
            input_keys = ['robots'],
            )

        self.robot_name = robot_name
        
        
        self.q = q
        self.t = t
        self.qdot_max_factor = qdot_max_factor
        self.qddot_max_factor = qddot_max_factor
        
        
    def on_enter(self, userdata):
        
        
        userdata.robots[self.robot_name].JMove(
            q = self.q,t = self.t,qdot_max_factor = self.qdot_max_factor,qddot_max_factor = self.qddot_max_factor)
        

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass