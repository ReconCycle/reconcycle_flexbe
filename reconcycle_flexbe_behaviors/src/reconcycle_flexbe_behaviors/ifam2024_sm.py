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
		# x:641 y:67, x:211 y:421
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.robots = None
		_state_machine.userdata.vision_utils = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:27 y:282, x:544 y:133, x:561 y:195, x:507 y:35, x:526 y:76, x:15 y:419, x:663 y:298
		_sm_concurrent_cell_init_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['robots', 'vision_utils'], output_keys=['robots', 'vision_utils'], conditions=[
										('finished', [('init_basler', 'continue'), ('init_realsense', 'continue'), ('init_panda_1', 'continue'), ('init_panda_2', 'continue')]),
										('failed', [('init_basler', 'failed')]),
										('failed', [('init_realsense', 'failed')]),
										('failed', [('init_panda_1', 'failed')]),
										('failed', [('init_panda_2', 'failed')])
										])

		with _sm_concurrent_cell_init_0:
			# x:192 y:45
			OperatableStateMachine.add('init_panda_1',
										InitPandaState(robot_name='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:210 y:106
			OperatableStateMachine.add('init_panda_2',
										InitPandaState(robot_name='panda_2'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})

			# x:205 y:249
			OperatableStateMachine.add('init_realsense',
										InitVisionUtilsState(camera_name='basler', camera_detections_topic='/vision/basler/detections', camera_table_name=None),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})

			# x:202 y:173
			OperatableStateMachine.add('init_basler',
										InitVisionUtilsState(camera_name='basler', camera_detections_topic='/vision/basler/detections', camera_table_name='table_vision'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'vision_utils': 'vision_utils'})



		with _state_machine:
			# x:176 y:56
			OperatableStateMachine.add('Concurrent_cell_init',
										_sm_concurrent_cell_init_0,
										transitions={'finished': 'wait', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'robots': 'robots', 'vision_utils': 'vision_utils'})

			# x:420 y:58
			OperatableStateMachine.add('wait',
										WaitState(wait_time=5),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
