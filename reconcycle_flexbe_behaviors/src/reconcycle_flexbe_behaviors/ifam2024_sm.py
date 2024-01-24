#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
from rbs_flexbe_states.init_panda_state import InitPandaState
from rbs_flexbe_states.jmove_state import JMoveState
from reconcycle_flexbe_states.init_cnc_state import InitCNCState
from reconcycle_flexbe_states.init_vision_utils_state import InitVisionUtilsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

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
		# x:1171 y:91, x:211 y:421
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.robots = None
		_state_machine.userdata.vision_utils = None
		_state_machine.userdata.cnc_client = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:27 y:282, x:544 y:133, x:561 y:195, x:613 y:369, x:601 y:239, x:567 y:309, x:576 y:444, x:730 y:365
		_sm_concurrent_cell_init_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robots', 'vision_utils', 'cnc_client'], output_keys=['robots', 'vision_utils', 'cnc_client'], conditions=[
										('finished', [('init_panda_1', 'continue'), ('init_panda_2', 'continue'), ('init_basler', 'continue'), ('init_rs', 'continue'), ('init_cnc', 'continue')]),
										('failed', [('init_panda_1', 'failed')]),
										('failed', [('init_panda_2', 'failed')]),
										('failed', [('init_basler', 'failed')]),
										('failed', [('init_rs', 'failed')]),
										('failed', [('init_cnc', 'failed')])
										])

		with _sm_concurrent_cell_init_0:
			# x:212 y:43
			OperatableStateMachine.add('init_panda_1',
										InitPandaState(robot_name='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:204 y:382
			OperatableStateMachine.add('init_cnc',
										InitCNCState(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cnc_client': 'cnc_client'})

			# x:210 y:106
			OperatableStateMachine.add('init_panda_2',
										InitPandaState(robot_name='panda_2'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:211 y:270
			OperatableStateMachine.add('init_rs',
										InitVisionUtilsState(camera_name='realsense', camera_detections_topic='/vision/realsense/detections', camera_table_name=None),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})

			# x:211 y:202
			OperatableStateMachine.add('init_basler',
										InitVisionUtilsState(camera_name='basler', camera_detections_topic='/vision/basler/detections', camera_table_name='table_vision'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})



		with _state_machine:
			# x:176 y:56
			OperatableStateMachine.add('Concurrent_cell_init',
										_sm_concurrent_cell_init_0,
										transitions={'finished': 'p1_move_to_init', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robots': 'robots', 'vision_utils': 'vision_utils', 'cnc_client': 'cnc_client'})

			# x:446 y:46
			OperatableStateMachine.add('p1_move_to_init',
										JMoveState(robot_name="panda_1", q=[0.12272088397521683,  -1.3181167800849902,  -0.20597019964421703,  -2.5176138428587596,  -0.20126330022266803,  1.254942861398061,  0.8085149702917125], duration=2, qdot_max_factor=0.3, qddot_max_factor=0.3),
										transitions={'continue': 'p2_move_to_init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:583 y:42
			OperatableStateMachine.add('p2_move_to_init',
										JMoveState(robot_name="panda_2", q=[-0.85320616, -0.613722  , -0.008163  , -2.44418   , -0.044385  ,         1.850346  ,  0.712032  ], duration=2, qdot_max_factor=0.3, qddot_max_factor=0.3),
										transitions={'continue': 'wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:806 y:48
			OperatableStateMachine.add('wait',
										WaitState(wait_time=3),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
