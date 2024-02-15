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
from rbs_flexbe_states.cmovefor_state import CMoveForState
from rbs_flexbe_states.init_reconcycle_panda_state import InitReconcyclePandaState
from rbs_flexbe_states.jmove_state import JMoveState
from rbs_flexbe_states.jpath_state import JPathState
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.cnc_cut_state import CncCutState
from reconcycle_flexbe_states.detection_to_disassembly_object_state import DetectionToDisassemblyObjectState
from reconcycle_flexbe_states.drop_object_state import DropObjectState
from reconcycle_flexbe_states.find_clear_drop_pose import FindClearDropPoseState
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
class IFAM2024SM(Behavior):
	'''
	IFAM 2024 demo
	'''


	def __init__(self):
		super(IFAM2024SM, self).__init__()
		self.name = 'IFAM2024'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1034 y:690, x:70 y:138, x:325 y:700
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'failed_during_cycle'])
		_state_machine.userdata.robots = None
		_state_machine.userdata.vision_utils = None
		_state_machine.userdata.cnc_client = None
		_state_machine.userdata.detection = None
		_state_machine.userdata.disassembly_object = None
		_state_machine.userdata.drop_pose_in_robot_frame = None

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


		# x:36 y:703, x:737 y:739
		_sm_place_smoke_detector_into_cnc_1 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['robots', 'disassembly_object'])

		with _sm_place_smoke_detector_into_cnc_1:
			# x:30 y:40
			OperatableStateMachine.add('Move above pickup table',
										JMoveState(robot_name="panda_1", q=rospy.get_param('/pose_db/demo_poses/smoke_detector_pickup_q'), duration=2.2, qdot_max_factor=1, qddot_max_factor=1),
										transitions={'continue': 'Relative pickup smoke detector', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:335 y:362
			OperatableStateMachine.add('Drop smoke detector to CNC',
										DropObjectState(robot_name="panda_1", drop_pose=[0.4991899990939808, 0.12168477043035125, 0.18724904468415368], drop_tf_name=None, move_time_to_above=3, move_time_to_below=1, offset_above_z=0.11),
										transitions={'continue': 'Depart', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:246 y:240
			OperatableStateMachine.add('Move above CNC',
										JPathState(robot_name="panda_1", move_path="a", duration=2.8),
										transitions={'continue': 'Drop smoke detector to CNC', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:540 y:592
			OperatableStateMachine.add('Move robot away (to init)',
										JMoveState(robot_name="panda_1", q=rospy.get_param('/pose_db/q_init/panda_1'), duration=2, qdot_max_factor=1, qddot_max_factor=1),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:124 y:129
			OperatableStateMachine.add('Relative pickup smoke detector',
										PickupObjectState(robot_name="panda_1", move_above_z=0.15),
										transitions={'continue': 'Move above CNC', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:416 y:467
			OperatableStateMachine.add('Depart',
										CMoveForState(robot_name="panda_1", dx=[0,0,0.03], t=1.5),
										transitions={'continue': 'Move robot away (to init)', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})


		# x:583 y:54, x:284 y:260, x:280 y:107, x:330 y:518, x:430 y:518
		_sm_move_robots_to_initial_configuration_2 = ConcurrencyContainer(outcomes=['continue', 'failed'], input_keys=['robots'], conditions=[
										('failed', [('p2_move_to_init', 'failed')]),
										('continue', [('p1_move_to_init', 'continue'), ('p2_move_to_init', 'continue')]),
										('failed', [('p1_move_to_init', 'failed')])
										])

		with _sm_move_robots_to_initial_configuration_2:
			# x:87 y:42
			OperatableStateMachine.add('p1_move_to_init',
										JMoveState(robot_name="panda_1", q=[0.12272088397521683,  -1.3181167800849902,  -0.20597019964421703,  -2.5176138428587596,  -0.20126330022266803,  1.254942861398061,  0.8085149702917125], duration=2, qdot_max_factor=0.3, qddot_max_factor=0.3),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:82 y:163
			OperatableStateMachine.add('p2_move_to_init',
										JMoveState(robot_name="panda_2", q=[-0.85320616, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,         1.850346  ,  0.712032  ], duration=2, qdot_max_factor=0.3, qddot_max_factor=0.3),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})


		# x:27 y:282, x:544 y:133, x:566 y:497, x:566 y:310, x:583 y:384, x:574 y:549, x:576 y:444
		_sm_init_workcell_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robots', 'vision_utils'], output_keys=['robots', 'vision_utils'], conditions=[
										('finished', [('init_panda_2', 'continue'), ('init_panda_1', 'continue'), ('Init basler', 'continue'), ('Init realsense', 'continue')]),
										('failed', [('init_panda_1', 'failed')]),
										('failed', [('init_panda_2', 'failed')]),
										('failed', [('Init basler', 'failed')]),
										('failed', [('Init realsense', 'failed')])
										])

		with _sm_init_workcell_3:
			# x:223 y:19
			OperatableStateMachine.add('init_panda_1',
										InitReconcyclePandaState(robot_name="panda_1", tool_name="ThreeJawChuck", use_toolchanger=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:232 y:388
			OperatableStateMachine.add('Init realsense',
										InitVisionUtilsState(camera_name='realsense', camera_detections_topic='/vision/realsense/detections', camera_table_name=None),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})

			# x:230 y:151
			OperatableStateMachine.add('init_panda_2',
										InitReconcyclePandaState(robot_name="panda_2", tool_name="VariableStiffnessGripper", use_toolchanger=False),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:236 y:294
			OperatableStateMachine.add('Init basler',
										InitVisionUtilsState(camera_name='basler', camera_detections_topic='/vision/basler/detections', camera_table_name='table_vision'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})


		# x:30 y:518, x:130 y:518
		_sm_check_battery_rotation_4 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['vision_utils', 'robots'])

		with _sm_check_battery_rotation_4:
			# x:30 y:40
			OperatableStateMachine.add('Move robot with inhand camera',
										JMoveState(robot_name="panda_2", q=None, duration=None, qdot_max_factor=None, qddot_max_factor=None),
										transitions={'continue': 'Get battery orientation', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:356 y:372
			OperatableStateMachine.add('Return to previous',
										JMoveState(robot_name="panda_2", q=None, duration=None, qdot_max_factor=None, qddot_max_factor=None),
										transitions={'continue': 'Move robot with inhand camera', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:183 y:190
			OperatableStateMachine.add('Get battery orientation',
										GetVisionResultState(camera_name="realsense", object_to_find='firealarm', timeout=3),
										transitions={'continue': 'Return to previous', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils', 'detection': 'detection'})



		with _state_machine:
			# x:153 y:38
			OperatableStateMachine.add('Init workcell',
										_sm_init_workcell_3,
										transitions={'finished': 'Move robots to initial configuration', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robots': 'robots', 'vision_utils': 'vision_utils'})

			# x:505 y:412
			OperatableStateMachine.add('Check battery rotation',
										_sm_check_battery_rotation_4,
										transitions={'failed': 'wait', 'continue': 'Cut smoke detector'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'vision_utils': 'vision_utils', 'robots': 'robots'})

			# x:510 y:499
			OperatableStateMachine.add('Cut smoke detector',
										CncCutState(),
										transitions={'continue': 'Pick smoke detector from CNC', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'disassembly_object': 'disassembly_object', 'cnc_client': 'cnc_client'})

			# x:532 y:706
			OperatableStateMachine.add('Drop smoke detector on table',
										FindClearDropPoseState(dropped_object_diameter=0.14, drop_limit_bbox=[[0.15,0.45],[0.45,0.15]], detections_parent_frame="vision_table_zero", robot_parent_frame='panda_1_link0'),
										transitions={'continue': 'Execute once again?', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detections': 'detection', 'drop_pose_in_robot_frame': 'drop_pose_in_robot_frame'})

			# x:937 y:373
			OperatableStateMachine.add('Execute once again?',
										OperatorDecisionState(outcomes=["yes","no"], hint="Execute once again?", suggestion="Repeat the cycle"),
										transitions={'yes': 'Move robots to initial configuration', 'no': 'finished'},
										autonomy={'yes': Autonomy.Off, 'no': Autonomy.Off})

			# x:507 y:164
			OperatableStateMachine.add('Get smoke detector type',
										GetVisionResultState(camera_name="basler", object_to_find='firealarm', timeout=3),
										transitions={'continue': 'Set disassembly params', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils', 'detection': 'detection'})

			# x:503 y:83
			OperatableStateMachine.add('Move robots to initial configuration',
										_sm_move_robots_to_initial_configuration_2,
										transitions={'continue': 'Get smoke detector type', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robots': 'robots'})

			# x:508 y:591
			OperatableStateMachine.add('Pick smoke detector from CNC',
										PickupObjectState(robot_name="panda_1", move_above_z=0.15),
										transitions={'continue': 'Drop smoke detector on table', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:507 y:323
			OperatableStateMachine.add('Place smoke detector into CNC',
										_sm_place_smoke_detector_into_cnc_1,
										transitions={'failed': 'wait', 'continue': 'Additional steps'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'robots': 'robots', 'disassembly_object': 'disassembly_object'})

			# x:43 y:350
			OperatableStateMachine.add('Robot error recovery',
										_sm_robot_error_recovery_0,
										transitions={'continue': 'Move robots to initial configuration', 'failed': 'failed_during_cycle'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:506 y:241
			OperatableStateMachine.add('Set disassembly params',
										DetectionToDisassemblyObjectState(),
										transitions={'continue': 'Place smoke detector into CNC', 'failed': 'wait'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'detection': 'detection', 'disassembly_object': 'disassembly_object'})

			# x:277 y:366
			OperatableStateMachine.add('wait',
										WaitState(wait_time=1),
										transitions={'done': 'Robot error recovery'},
										autonomy={'done': Autonomy.Off})

			# x:758 y:381
			OperatableStateMachine.add('Additional steps',
										DecisionState(outcomes=["rotate_gcode", "nothing"], conditions=lambda obj: "rotate_gcode" if obj.name == "hekatron" else "nothing"),
										transitions={'rotate_gcode': 'Check battery rotation', 'nothing': 'Cut smoke detector'},
										autonomy={'rotate_gcode': Autonomy.Off, 'nothing': Autonomy.Off},
										remapping={'input_value': 'disassembly_object'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]