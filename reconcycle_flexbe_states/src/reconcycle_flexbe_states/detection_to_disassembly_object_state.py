
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import SmokeDetector

class DetectionToDisassemblyObjectState(EventState):

    def __init__(self):
        super(DetectionToDisassemblyObjectState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['detection'],
                                            output_keys = ['disassembly_object'])
        
        self.out = 'failed'

    def on_enter(self, userdata):
        
        detection = userdata.detection   
            
        #handle hcas with if elif else ... 
        userdata.disassembly_object = SmokeDetector(detection)

        self.out = 'continue'
        return self.out

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass