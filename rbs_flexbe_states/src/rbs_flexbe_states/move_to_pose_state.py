#!/usr/bin/python

from flexbe_core import EventState
import json

from robotblockset_python.panda_ros import panda_ros
from robotblockset_python.transformations import *
from robotblockset_python.ros.grippers_ros import ToolChanger, ThreeJawChuck, dualVacuumGripper

from disassembly_pipeline.utils.disassembly_manager import DisassemblyManager
from disassembly_pipeline.utils.robot_quick_init import DummyRobot

class MoveToPoseState(EventState):

    def __init__(self, robot_name, pose_db_entry, pose, offset, pose_db_path, motion_duration):
        super(MoveToPoseState, self).__init__(outcomes = ['continue', 'failed'])

        pose_database_file = open(pose_db_path)
        self.robot_name = robot_name
        self.pose_db_entry = pose_db_entry
        self.pose = pose
        self.offset = offset
        self.pose_db = json.load(pose_database_file)
        self.motion_duration = motion_duration

    def on_enter(self, userdata):
        if (self.pose_db_entry is not None) and (self.pose is not None):
            raise ValueError('Either pose_db_entry or pose must be specified, not both')
        elif self.pose_db_entry is not None:
            entry = userdata.pose_db_entry.split('/')
            pose = self.pose_db[entry[0]][entry[1]]['pose']
        elif self.pose is not None:
            pose = self.pose
        else:
            raise ValueError('Either pose_db_entry or pose must be specified')

        if self.offset is not None:
            assert type(self.offset)==list and len(self.offset)==3, 'Offset must be a list of 3 numbers'
            pose[0:3] += self.offset

        userdata.robots[self.robot_name].CMove(pose, t=self.motion_duration)
    
    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass
