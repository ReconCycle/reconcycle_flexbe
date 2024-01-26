
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import SmokeDetector

class DropObjectState(EventState):

    def __init__(self):
        super(DropObjectState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['disassembly_object'])
        
        self.out = 'failed'

    def on_enter(self, userdata):
        
        #TODO glede na userdata.object
        # open gripper

        self.out = 'continue'
        return self.out

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass