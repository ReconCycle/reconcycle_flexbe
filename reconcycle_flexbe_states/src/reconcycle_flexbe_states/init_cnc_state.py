#!/usr/bin/python

from flexbe_core import EventState, Logger
from disassembly_pipeline.cnc_manager.src.action_client_cnc import CNCActionClient

class InitCNCState(EventState):

    def __init__(self):
        super(InitCNCState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['cnc_client'],
                                            output_keys = ['cnc_client'])
        
        self.cnc_client = CNCActionClient(wait_for_server=False, init_ros_node=False)
        self.out = 'continue'
        pass

    def on_enter(self, userdata):
        
        userdata.cnc_client = self.cnc_client
        self.out = 'continue'
        return self.out

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass