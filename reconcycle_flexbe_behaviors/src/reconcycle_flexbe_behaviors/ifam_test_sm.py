#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from rbs_flexbe_states.init_panda_state import InitPandaState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jan 22 2024
@author: Boris Kuster
'''
class IFAM_testSM(Behavior):
	'''
	testing state by state
	'''


	def __init__(self):
		super(IFAM_testSM, self).__init__()
		self.name = 'IFAM_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:518, x:130 y:518
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.robots = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:303 y:130
			OperatableStateMachine.add('init_p1',
										InitPandaState(robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'robots': 'robots'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
