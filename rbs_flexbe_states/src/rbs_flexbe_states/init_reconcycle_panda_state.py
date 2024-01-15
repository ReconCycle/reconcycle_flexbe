#!/usr/bin/python

from flexbe_core import EventState, Logger
import numpy as np
from disassembly_pipeline.utils.robot_quick_init import initialize_robot

class InitReconcyclePandaState(EventState):
    '''
    Initializes the Panda robot.
    -- robot_name   string Name of the robot.
    <= continue     Continue with execution.
    <= failed       Failed to initialize the robot.
    '''
    
    
    def __init__(self, robot_name, tool_name, use_toolchanger = False):
        super(InitReconcyclePandaState, self).__init__(outcomes = ['continue', 'failed'], 
                                             input_keys = ['robots'],
                                             output_keys = ['robots'])

        self.robot_name = robot_name
        self.tool_name = tool_name
        self.use_toolchanger = use_toolchanger

    def on_enter(self, userdata):
        
        if not hasattr(userdata, 'robots'):
            userdata.robots = dict()
        
        try:
            robot = initialize_robot(robot_name = self.robot_name,
                     tool_name = self.tool_name,
                     start_controller = 'position_joint_trajectory_controller',
                     collision_thresholds = {"F": 50, "T": 20, "tq": 30},
                     gripper_init_kwargs = {},
                     toolchanger = self.use_toolchanger,
                     toolchanger_init_kwargs = {})

            userdata.robots[self.robot_name] = robot
            return 'continue'
        except Exception as e:
            print(e)
            return 'failed'

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass
