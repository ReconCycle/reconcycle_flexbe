
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import SmokeDetector

class CncCutState(EventState):

    def __init__(self):
        super(CncCutState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['disassembly_object','cnc_client'])
        
        self.out = 'failed'

    def on_enter(self, userdata):
        
        #todo
        
        self.out = 'continue'
        return self.out

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass