
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import SmokeDetector
from disassembly_pipeline.utils.find_clear_drop_pose import find_drop_pose
from disassembly_pipeline.utils.tf_utils import TFManager
from robotblockset_python.transformations import x2t
import copy
import numpy as np

class FindClearDropPoseState(EventState):

    def __init__(self, dropped_object_diameter = 0.14,
                       drop_limit_bbox = [[0.15,0.45], [0.45,0.15]],
                       detections_parent_frame = "vision_table_zero",
                       drop_pose_z_in_robot_frame = 0.1,
                       robot_parent_frame = 'panda_1_link0'):
        super(FindClearDropPoseState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['detections'],
                                            output_keys = ['drop_pose_in_robot_frame'])
        
        self.dropped_object_diameter = dropped_object_diameter
        self.drop_limit_bbox = drop_limit_bbox
        self.detections_parent_frame = detections_parent_frame
        self.drop_pose_z_in_robot_frame = drop_pose_z_in_robot_frame
        self.robot_parent_frame = robot_parent_frame

        self.tf_manager = TFManager()
        self.sendTf = self.tf_manager.SendTransform2tf
        self.tf2x = self.tf_manager.tf2x

        self.out = 'failed'

    def on_enter(self, userdata):

        other_detections = userdata.detections
        #other_detections = copy.deepcopy(VU.detections)
        
        input_obb = [[0,0],[self.dropped_object_diameter,0],[self.dropped_object_diameter, self.dropped_object_diameter],[0, self.dropped_object_diameter]]
        other_detections = [d.obb for d in other_detections]
        drop_position = find_drop_pose(input_obb= input_obb,
                        other_objects_bbox = other_detections, 
                        limit_bbox= self.drop_limit_bbox,
                        sendTf = self.sendTf,
                        parent_frame = self.detections_parent_frame
                        )
        drop_T_in_table_frame = np.eye(4) # rotation will be overwritten later
        drop_T_in_table_frame[0:2,-1] = drop_position
        drop_T_in_robot_frame = x2t(self.tf2x(self.robot_parent_frame, self.detections_parent_frame))@drop_T_in_table_frame
        drop_T_in_robot_frame[2, -1] = self.drop_pose_z_in_robot_frame
        
        drop_x, drop_y, drop_z = drop_T_in_robot_frame[0:3, -1]

        userdata.drop_pose_in_robot_frame = [drop_x, drop_y, drop_z]

        self.out = 'continue'

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass