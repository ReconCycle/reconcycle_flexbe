#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Oct 04 2021
@author: Miha with the help of Boris and RT
'''
class EMOwhenweareawaySM(Behavior):
	'''
	A challenge posed by Primoz
	'''


	def __init__(self):
		super(EMOwhenweareawaySM, self).__init__()
		self.name = 'EMO when we are away'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1214 y:306, x:818 y:665
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.open_conf = [0.2]
		_state_machine.userdata.close_conf = [0.8]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:273 y:266
			OperatableStateMachine.add('Open',
										MoveSoftHand(motion_duration=2, motion_timestep=0.01),
										transitions={'continue': 'Close', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_conf', 'success': 'success'})

			# x:606 y:214
			OperatableStateMachine.add('Close',
										MoveSoftHand(motion_duration=2, motion_timestep=0.01),
										transitions={'continue': 'Open', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'close_conf', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
