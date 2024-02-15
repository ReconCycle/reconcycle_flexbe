#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
import rospy
import rosparam

class UploadPoseDBState(EventState):

    '''
    
    '''

    def __init__(self, posedb_file_location = '../disassembly_pipeline/poses/pose_database.json', ros_param_name = 'pose_db'):
        super( UploadPoseDBState, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['success'])
        self.posedb_file_location = posedb_file_location
        self.ros_param_name = ros_param_name
            
        self.out = 'failed'

    def on_enter(self, userdata):
                
        param_dict = rosparam.load_file(self.posedb_file_location, self.ros_param_name)
        tmp = param_dict[0][0]
        rosparam.upload_params(ns = self.ros_param_name, values = tmp)
        self.out = 'continue'

    def execute(self, userdata):
        return self.out
    
    def on_exit(self, userdata):

        return 'continue'
 
if __name__ == '__main__':

    print("Testing standalone")
