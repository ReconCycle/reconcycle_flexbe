#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_states.read_from_mongodb_POSE import ReadFromMongoPOSE
from reconcycle_flexbe_states.write_to_mongodb_pose import WriteToMongoPOSE
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 14 2021
@author: Rok Pahic...
'''
class EMOdemo2SM(Behavior):
	'''
	Demo za EMO:
Preberi pozicijo HCA iz kamere
Poberi z Qb hand
Daj v primez
Stisni in zavrti tja in nazaj
Spusti primez 
Poberi z robotm
Odlozi nazaj na zacetek
Ponovi
	'''


	def __init__(self):
		super(EMOdemo2SM, self).__init__()
		self.name = 'EMO demo 2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.position1 = [0.3,0,0.3]
		_state_machine.userdata.orentation1 = [0.92,-0.38,0]
		_state_machine.userdata.name = "test_pose"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:192 y:111
			OperatableStateMachine.add('test',
										WriteToMongoPOSE(),
										transitions={'continue': 'read pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'position': 'position1', 'orientation': 'orentation1', 'entry_name': 'name'})

			# x:483 y:292
			OperatableStateMachine.add('read pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'name', 'joints_data': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
