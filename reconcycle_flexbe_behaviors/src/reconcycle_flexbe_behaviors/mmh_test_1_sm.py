#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_behaviors.generic_disassembly_protocol_sm import GenericdisassemblyprotocolSM
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 21 2022
@author: Matevz M.H.
'''
class MMH_Test_1SM(Behavior):
	'''
	First attempt to flexbe
	'''


	def __init__(self):
		super(MMH_Test_1SM, self).__init__()
		self.name = 'MMH_Test_1'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(GenericdisassemblyprotocolSM, 'Generic disassembly protocol')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.value = 5

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:25
			OperatableStateMachine.add(''Error recovery'',
										FrankaErrorRecoveryActionProxy(robot_name='panda_2'),
										transitions={'continue': 'Load Cartesian controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:515 y:337
			OperatableStateMachine.add('Generic disassembly protocol',
										self.use_behavior(GenericdisassemblyprotocolSM, 'Generic disassembly protocol'),
										transitions={'finished': 'failed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:234 y:23
			OperatableStateMachine.add('Load Cartesian controllers',
										LoadControllerProxyClient(desired_controller='cartesian_imepance_controller', robot_name='panda_2'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
