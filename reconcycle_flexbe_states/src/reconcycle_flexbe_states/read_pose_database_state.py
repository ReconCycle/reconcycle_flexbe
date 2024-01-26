
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import SmokeDetector
import rospy

class ReadPoseDataBaseState(EventState):

    def __init__(self, entry_name):
        super(ReadPoseDataBaseState, self).__init__(outcomes = ['continue', 'failed'],
                                            output_keys = ['pose_db'])
        
        self.out = 'failed'

    def on_enter(self, userdata):
        
           
        userdata.pose_db = rospy.get_param("/pose_db")

        self.out = 'continue'
        return self.out

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass