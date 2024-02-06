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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jan 29 2024
@author: test
'''
class testSM(Behavior):
	'''
	test
	'''


	def __init__(self):
		super(testSM, self).__init__()
		self.name = 'test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:518, x:927 y:192
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.test = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:351 y:177
			OperatableStateMachine.add('test',
										OperatorDecisionState(outcomes=["ja","ne"], hint="hint", suggestion="ja"),
										transitions={'ja': 'dec', 'ne': 'failed'},
										autonomy={'ja': Autonomy.Full, 'ne': Autonomy.Full})

			# x:607 y:300
			OperatableStateMachine.add('dec',
										DecisionState(outcomes=["1","2"], conditions=lambda x: x),
										transitions={'1': 'finished', '2': 'failed'},
										autonomy={'1': Autonomy.Off, '2': Autonomy.Off},
										remapping={'input_value': 'test'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
