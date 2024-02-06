
from flexbe_core import EventState, Logger
from disassembly_pipeline.environment.types import SmokeDetector
from disassembly_pipeline.utils.find_clear_drop_pose import find_drop_pose
from disassembly_pipeline.utils.tf_utils import TFManager
from robotblockset_python.transformations import x2t
import copy
import numpy as np

class DropObjectState(EventState):

    def __init__(self, robot_name, drop_pose, drop_tf_name, move_time_to_above, move_time_to_below, offset_above_z):
        super(DropObjectState, self).__init__(outcomes = ['continue', 'failed'],
                                            input_keys = ['robots', 'drop_pose_tf'],
                                            )
        

        self.robot_name = robot_name
        self.drop_tf_name = drop_tf_name
        self.drop_pose = drop_pose
        self.move_time_to_above = move_time_to_above
        self.move_time_to_below = move_time_to_below
        self.offset_above_z = offset_above_z

        self.tf_manager = TFManager()
        self.sendTf = self.tf_manager.SendTransform2tf
        self.tf2x = self.tf_manager.tf2x

        self.out = 'failed'

    def on_enter(self, userdata):

        robot = userdata.robots[self.robot_name]

        #drop_pose_x_below_tf = userdata.drop_pose_tf

        #drop_pose_x_below = self.tf2x(parent_frame = robot.Base_link_name, child_frame = drop_pose_x_below_tf)

        if (self.drop_pose is None) and (self.drop_tf_name is not None):
            # We get drop pose tf name as input parameter and lookup the transform
            drop_pose_x_below = self.tf2x(parent_frame = robot.Base_link_name, child_frame = self.drop_tf)
        elif (self.drop_pose is not None) and (self.drop_tf_name is None):
            # We get drop pose in robot frame as input parameter
            drop_pose_x_below = self.drop_pose
        elif (self.drop_pose is None) and (self.drop_tf_name is None):
            # We want to get the drop pose in robot frame from userdata
            drop_pose_x_below = userdata.drop_pose_tf
            
        # First we will move above the drop pose, then down, then back above
        
        # Construct pose above the drop pose 
        drop_pose_x_above = copy.deepcopy(drop_pose_x_below)
        drop_pose_x_above[2] += self.offset_above_z

        robot.error_recovery()
        robot.CMove(drop_pose_x_above, self.move_time_to_above)
        robot.CMove(drop_pose_x_below, self.move_time_to_below)

        robot.gripper.open()

        robot.CMove(x = drop_pose_x_above, t = self.move_time_to_above)

        self.out = 'continue'

    def execute(self, userdata):
        return self.out

    def on_exit(self, userdata):
        pass