#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Feb 04 2021
@author: Rok Pahic
'''
class TestchangetoolSM(Behavior):
	'''
	Testing state for raspberry interaction
	'''


	def __init__(self):
		super(TestchangetoolSM, self).__init__()
		self.name = 'Test change tool'

		# parameters of this behavior
		self.add_parameter('name_of_service', 'tool_changer_unlock')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.value1 = True
		_state_machine.userdata.value2 = False
		_state_machine.userdata.name = 'tool_service_unlock'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:305 y:54
			OperatableStateMachine.add('close_tool',
										ActivateRaspiDigitalOuput(service_name=self.name_of_service),
										transitions={'continue': 'test', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'value2', 'success': 'success'})

			# x:639 y:181
			OperatableStateMachine.add('test',
										ActivateRaspiDigitalOuput(service_name=self.name_of_service),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'value1', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
