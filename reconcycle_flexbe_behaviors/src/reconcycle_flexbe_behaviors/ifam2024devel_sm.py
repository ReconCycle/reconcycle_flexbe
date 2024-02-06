#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.decision_state import DecisionState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.wait_state import WaitState
from rbs_flexbe_states.cmove_state import CMoveState
from rbs_flexbe_states.init_reconcycle_panda_state import InitReconcyclePandaState
from rbs_flexbe_states.jmove_state import JMoveState
from rbs_flexbe_states.jpath_state import JPathState
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.cnc_cut_state import CncCutState
from reconcycle_flexbe_states.drop_object_state import DropObjectState
from reconcycle_flexbe_states.find_clear_drop_pose import FindClearDropPoseState
from reconcycle_flexbe_states.find_cnc_battery_orientation import FindCNCBatteryRotation
from reconcycle_flexbe_states.get_vision_result_state import GetVisionResultState
from reconcycle_flexbe_states.init_vision_utils_state import InitVisionUtilsState
from reconcycle_flexbe_states.pickup_object_state import PickupObjectState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import rospy
# [/MANUAL_IMPORT]


'''
Created on Mon Jan 15 2024
@author: BK, MS
'''
class IFAM2024DEVELSM(Behavior):
	'''
	IFAM 2024 demo
	'''


	def __init__(self):
		super(IFAM2024DEVELSM, self).__init__()
		self.name = 'IFAM2024DEVEL'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1222 y:452, x:50 y:192, x:248 y:945
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'failed_during_cycle'])
		_state_machine.userdata.robots = None
		_state_machine.userdata.vision_utils = None
		_state_machine.userdata.cnc_client = None
		_state_machine.userdata.detection = None
		_state_machine.userdata.disassembly_object = None
		_state_machine.userdata.drop_pose_in_robot_frame = None
		_state_machine.userdata.drop_pose_tf = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:420 y:91, x:24 y:300, x:230 y:518, x:330 y:518
		_sm_robot_error_recovery_0 = ConcurrencyContainer(outcomes=['continue', 'failed'], conditions=[
										('continue', [('Panda 1 Error Recovery', 'continue'), ('Panda 2 Error Recovery', 'continue')]),
										('failed', [('Panda 2 Error Recovery', 'failed'), ('Panda 1 Error Recovery', 'failed')])
										])

		with _sm_robot_error_recovery_0:
			# x:123 y:50
			OperatableStateMachine.add('Panda 1 Error Recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:122 y:138
			OperatableStateMachine.add('Panda 2 Error Recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_2"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:36 y:703, x:773 y:644
		_sm_place_smoke_detector_into_cnc_1 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['robots', 'disassembly_object', 'drop_pose_tf'])

		with _sm_place_smoke_detector_into_cnc_1:
			# x:56 y:36
			OperatableStateMachine.add('Move above pickup table',
										JMoveState(robot_name="panda_1", q=rospy.get_param('/pose_db/demo_poses/smoke_detector_pickup_q'), t=3, qdot_max_factor=1, qddot_max_factor=1),
										transitions={'continue': 'Relative pickup smoke detector', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:331 y:350
			OperatableStateMachine.add('Drop fumonic',
										DropObjectState(robot_name="panda_1", drop_pose=rospy.get_param('/pose_db/panda_1/cnc_fumonic_chuck/pose'), drop_tf_name=None, move_time_to_above=1.5, move_time_to_below=1, offset_above_z=0.11),
										transitions={'continue': 'Move robot away (to init)', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'drop_pose_tf': 'drop_pose_tf'})

			# x:560 y:355
			OperatableStateMachine.add('Drop hekatron to CNC',
										DropObjectState(robot_name="panda_1", drop_pose=rospy.get_param('/pose_db/panda_1/cnc_hekatron_chuck/pose'), drop_tf_name=None, move_time_to_above=1.5, move_time_to_below=1, offset_above_z=0.11),
										transitions={'continue': 'Move robot away (to init)', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'drop_pose_tf': 'drop_pose_tf'})

			# x:485 y:155
			OperatableStateMachine.add('Move above CNC',
										JPathState(robot_name="panda_1", path=[rospy.get_param('/pose_db/panda_1/via_cnc_vision/joints'), rospy.get_param('/pose_db/panda_1/above_cnc/joints')], t=4),
										transitions={'continue': 'Determine drop pose tf name', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:583 y:562
			OperatableStateMachine.add('Move robot away (to init)',
										JMoveState(robot_name="panda_1", q=rospy.get_param('/pose_db/q_init/panda_1'), t=3, qdot_max_factor=1, qddot_max_factor=1),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:285 y:54
			OperatableStateMachine.add('Relative pickup smoke detector',
										PickupObjectState(robot_name="panda_1", pick_up_pose=None, move_above_z=0.15),
										transitions={'continue': 'Move above CNC', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:460 y:241
			OperatableStateMachine.add('Determine drop pose tf name',
										DecisionState(outcomes=['drop_hekatron', 'drop_fumonic'], conditions=lambda x: 'drop_hekatron' if (x.detailed_class=='hekatron') else 'drop_fumonic'),
										transitions={'drop_hekatron': 'Drop hekatron to CNC', 'drop_fumonic': 'Drop fumonic'},
										autonomy={'drop_hekatron': Autonomy.Off, 'drop_fumonic': Autonomy.Off},
										remapping={'input_value': 'disassembly_object'})


		# x:583 y:54, x:284 y:260, x:280 y:107, x:330 y:518, x:430 y:518
		_sm_move_robots_to_initial_configuration_2 = ConcurrencyContainer(outcomes=['continue', 'failed'], input_keys=['robots'], conditions=[
										('failed', [('p2_move_to_init', 'failed')]),
										('continue', [('p1_move_to_init', 'continue'), ('p2_move_to_init', 'continue')]),
										('failed', [('p1_move_to_init', 'failed')])
										])

		with _sm_move_robots_to_initial_configuration_2:
			# x:87 y:42
			OperatableStateMachine.add('p1_move_to_init',
										JMoveState(robot_name="panda_1", q=[0.12272088397521683,  -1.3181167800849902,  -0.20597019964421703,  -2.5176138428587596,  -0.20126330022266803,  1.254942861398061,  0.8085149702917125], t=3, qdot_max_factor=0.3, qddot_max_factor=0.3),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:82 y:163
			OperatableStateMachine.add('p2_move_to_init',
										JMoveState(robot_name="panda_2", q=[-0.85320616, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,         1.850346  ,  0.712032  ], t=3, qdot_max_factor=0.3, qddot_max_factor=0.3),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})


		# x:27 y:282, x:544 y:133, x:566 y:497, x:566 y:310, x:583 y:384
		_sm_init_workcell_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robots'], output_keys=['robots'], conditions=[
										('finished', [('init_panda_2', 'continue'), ('init_panda_1', 'continue')]),
										('failed', [('init_panda_1', 'failed')]),
										('failed', [('init_panda_2', 'failed')])
										])

		with _sm_init_workcell_3:
			# x:223 y:19
			OperatableStateMachine.add('init_panda_1',
										InitReconcyclePandaState(robot_name="panda_1", tool_name="ThreeJawChuck", use_toolchanger=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:230 y:151
			OperatableStateMachine.add('init_panda_2',
										InitReconcyclePandaState(robot_name="panda_2", tool_name="VariableStiffnessGripper", use_toolchanger=False),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})


		# x:542 y:46, x:14 y:136, x:509 y:187, x:595 y:220, x:0 y:232
		_sm_init_vision_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['vision_utils'], output_keys=['vision_utils'], conditions=[
										('finished', [('init basler', 'continue'), ('init realsense', 'continue')]),
										('failed', [('init basler', 'failed')]),
										('failed', [('init realsense', 'failed')])
										])

		with _sm_init_vision_4:
			# x:121 y:44
			OperatableStateMachine.add('init basler',
										InitVisionUtilsState(camera_name='basler', camera_detections_topic='/vision/basler/detections', camera_table_name='table_vision'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})

			# x:152 y:205
			OperatableStateMachine.add('init realsense',
										InitVisionUtilsState(camera_name='realsense', camera_detections_topic='/vision/realsense/detections', camera_table_name=None),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})


		# x:30 y:518, x:600 y:733
		_sm_check_battery_rotation_5 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['vision_utils', 'robots', 'disassembly_object'], output_keys=['disassembly_object'])

		with _sm_check_battery_rotation_5:
			# x:212 y:51
			OperatableStateMachine.add('move to side',
										JMoveState(robot_name="panda_2", q=[-0.85320616, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,1.850346  ,  0.712032  ], t=3, qdot_max_factor=0.7, qddot_max_factor=0.7),
										transitions={'continue': 'move above cnc', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:478 y:313
			OperatableStateMachine.add('find_battery_rotation',
										FindCNCBatteryRotation(robot_name="panda_2", vision_utils_name="realsense"),
										transitions={'continue': 'move to side 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'vision_utils': 'vision_utils', 'disassembly_object': 'disassembly_object'})

			# x:260 y:170
			OperatableStateMachine.add('move above cnc',
										JMoveState(robot_name="panda_2", q=[ 1.29354882, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,1.850346  ,  0.712032  ], t=3, qdot_max_factor=0.6, qddot_max_factor=0.6),
										transitions={'continue': 'cmove_above_chuck', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:466 y:594
			OperatableStateMachine.add('move to init',
										JMoveState(robot_name="panda_2", q=[-0.85320616, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,1.850346  ,  0.712032  ], t=3, qdot_max_factor=0.7, qddot_max_factor=0.7),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:430 y:447
			OperatableStateMachine.add('move to side 2',
										JMoveState(robot_name="panda_2", q=[ 1.29354882, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,1.850346  ,  0.712032  ], t=3, qdot_max_factor=0.6, qddot_max_factor=0.6),
										transitions={'continue': 'move to init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:480 y:179
			OperatableStateMachine.add('cmove_above_chuck',
										CMoveState(robot_name="panda_2", x=[0.157, 0.63, 0.34886717, 0, 0.70710678, 0.70710678, -0], t=2),
										transitions={'continue': 'find_battery_rotation', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})



		with _state_machine:
			# x:114 y:31
			OperatableStateMachine.add('Init vision',
										_sm_init_vision_4,
										transitions={'finished': 'Init workcell', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'vision_utils': 'vision_utils'})

			# x:505 y:412
			OperatableStateMachine.add('Check battery rotation',
										_sm_check_battery_rotation_5,
										transitions={'failed': 'wait', 'continue': 'Cut smoke detector'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'vision_utils': 'vision_utils', 'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:510 y:499
			OperatableStateMachine.add('Cut smoke detector',
										CncCutState(detailed_class=None, battery_rotation=None),
										transitions={'continue': 'Determine pick up pose', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'disassembly_object': 'disassembly_object', 'cnc_client': 'cnc_client'})

			# x:545 y:577
			OperatableStateMachine.add('Determine pick up pose',
										DecisionState(outcomes=["hekatron", "fumonic"], conditions=lambda x: "hekatron" if (x.detailed_class == "hekatron") else "fumonic"),
										transitions={'hekatron': 'Pick up hekatron', 'fumonic': 'Pick up fumonic'},
										autonomy={'hekatron': Autonomy.Off, 'fumonic': Autonomy.Off},
										remapping={'input_value': 'disassembly_object'})

			# x:1169 y:869
			OperatableStateMachine.add('Drop smoke detector on table',
										DropObjectState(robot_name="panda_1", drop_pose=None, drop_tf_name=None, move_time_to_above=3, move_time_to_below=1, offset_above_z=0.15),
										transitions={'continue': 'Move above vision table', 'failed': 'Pick up fumonic'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'drop_pose_tf': 'drop_pose_in_robot_frame'})

			# x:934 y:346
			OperatableStateMachine.add('Execute once again?',
										OperatorDecisionState(outcomes=["yes","no"], hint="Execute once again?", suggestion="yes"),
										transitions={'yes': 'Move robots to initial configuration', 'no': 'finished'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off})

			# x:615 y:866
			OperatableStateMachine.add('Find all objects on table',
										GetVisionResultState(camera_name='basler', object_to_find=None, timeout=3.5),
										transitions={'continue': 'Find clear drop pose', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils', 'detection': 'detection'})

			# x:819 y:870
			OperatableStateMachine.add('Find clear drop pose',
										FindClearDropPoseState(dropped_object_diameter=0.14, drop_limit_bbox=[[0.15,0.45],[0.45,0.15]], detections_parent_frame="vision_table_zero", drop_pose_z_in_robot_frame=0.07, robot_parent_frame='panda_1_link0'),
										transitions={'continue': 'Drop smoke detector on table', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detections': 'detection', 'drop_pose_in_robot_frame': 'drop_pose_in_robot_frame'})

			# x:539 y:193
			OperatableStateMachine.add('Get smoke detector type',
										GetVisionResultState(camera_name="basler", object_to_find='firealarm', timeout=3),
										transitions={'continue': 'Place smoke detector into CNC', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils', 'detection': 'disassembly_object'})

			# x:333 y:39
			OperatableStateMachine.add('Init workcell',
										_sm_init_workcell_3,
										transitions={'finished': 'Move robots to initial configuration', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robots': 'robots'})

			# x:1155 y:598
			OperatableStateMachine.add('Move above vision table',
										JMoveState(robot_name="panda_1", q=rospy.get_param('/pose_db/demo_poses/smoke_detector_pickup_q'), t=2, qdot_max_factor=0.5, qddot_max_factor=0.5),
										transitions={'continue': 'Execute once again?', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:503 y:83
			OperatableStateMachine.add('Move robots to initial configuration',
										_sm_move_robots_to_initial_configuration_2,
										transitions={'continue': 'Pick up fumonic', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robots': 'robots'})

			# x:491 y:693
			OperatableStateMachine.add('Pick up fumonic',
										PickupObjectState(robot_name="panda_1", pick_up_pose=rospy.get_param('/pose_db/panda_1/cnc_fumonic_chuck/pose'), move_above_z=0.15),
										transitions={'continue': 'Find all objects on table', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:654 y:689
			OperatableStateMachine.add('Pick up hekatron',
										PickupObjectState(robot_name="panda_1", pick_up_pose=rospy.get_param('/pose_db/panda_1/cnc_hekatron_chuck/pose'), move_above_z=0.15),
										transitions={'continue': 'Find all objects on table', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:507 y:323
			OperatableStateMachine.add('Place smoke detector into CNC',
										_sm_place_smoke_detector_into_cnc_1,
										transitions={'failed': 'wait', 'continue': 'Additional steps'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object', 'drop_pose_tf': 'drop_pose_tf'})

			# x:43 y:350
			OperatableStateMachine.add('Robot error recovery',
										_sm_robot_error_recovery_0,
										transitions={'continue': 'Move robots to initial configuration', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:254 y:502
			OperatableStateMachine.add('wait',
										WaitState(wait_time=1),
										transitions={'done': 'Robot error recovery'},
										autonomy={'done': Autonomy.Off})

			# x:780 y:417
			OperatableStateMachine.add('Additional steps',
										DecisionState(outcomes=["rotate_gcode", "nothing"], conditions=lambda x: "rotate_gcode" if (x.detailed_class == "hekatron") else "nothing"),
										transitions={'rotate_gcode': 'Check battery rotation', 'nothing': 'Cut smoke detector'},
										autonomy={'rotate_gcode': Autonomy.Off, 'nothing': Autonomy.Off},
										remapping={'input_value': 'disassembly_object'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
