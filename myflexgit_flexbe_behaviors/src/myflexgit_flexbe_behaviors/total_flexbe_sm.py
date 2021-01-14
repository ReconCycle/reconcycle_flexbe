#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from myflexgit_flexbe_states.CallAction_TF_Cart import CallT1
from myflexgit_flexbe_states.CallAction_TF_CartLin import CallT2
from myflexgit_flexbe_states.Read_TF_Cart import ReadT1
from myflexgit_flexbe_states.Read_TF_CartLin import ReadT2
from myflexgit_flexbe_states.call_joint_trap_vel_action_server import CallJointTrap
from myflexgit_flexbe_states.read_from_mongodb import ReadFromMongo
from myflexgit_flexbe_states.write_to_mongodb import WriteToMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Dec 12 2020
@author: Matej
'''
class TotalFlexBeSM(Behavior):
	'''
	All states
	'''


	def __init__(self):
		super(TotalFlexBeSM, self).__init__()
		self.name = 'Total FlexBe'

		# parameters of this behavior
		self.add_parameter('target1', '"target1"')
		self.add_parameter('world', '"world"')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:774 y:596, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.entry_name = 'matej'
		_state_machine.userdata.joints_data = []
		_state_machine.userdata.joint_values = []
		_state_machine.userdata.entry_data = []
		_state_machine.userdata.target1_frame = "target1"
		_state_machine.userdata.t1_data = []
		_state_machine.userdata.t1_out = []
		_state_machine.userdata.target2_frame = "target2"
		_state_machine.userdata.t2_data = []
		_state_machine.userdata.t2_out = []
		_state_machine.userdata.source_frame = "world"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:83 y:52
			OperatableStateMachine.add('WriteDB',
										WriteToMongo(),
										transitions={'continue': 'ReadDB', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_data': 'entry_data', 'entry_name': 'entry_name'})

			# x:722 y:187
			OperatableStateMachine.add('CallCartT1',
										CallT1(),
										transitions={'continue': 'ReadTarget2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t1_data': 't1_data', 't1_out': 't1_out'})

			# x:524 y:54
			OperatableStateMachine.add('CallJointTrap',
										CallJointTrap(),
										transitions={'continue': 'ReadTarget1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:307 y:52
			OperatableStateMachine.add('ReadDB',
										ReadFromMongo(),
										transitions={'continue': 'CallJointTrap', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'entry_name', 'joints_data': 'joints_data'})

			# x:722 y:54
			OperatableStateMachine.add('ReadTarget1',
										ReadT1(target_frame=target1, source_frame=world),
										transitions={'continue': 'CallCartT1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target1_frame': 'target1_frame', 'source_frame': 'source_frame', 't1_data': 't1_data'})

			# x:723 y:335
			OperatableStateMachine.add('ReadTarget2',
										ReadT2(),
										transitions={'continue': 'CallCartLinT2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target2_frame': 'target2_frame', 'source_frame': 'source_frame', 't2_data': 't2_data'})

			# x:723 y:455
			OperatableStateMachine.add('CallCartLinT2',
										CallT2(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 't2_data', 't2_out': 't2_out'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]