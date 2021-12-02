#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_states.CallAction_ForceAction import CallForceAction
from reconcycle_flexbe_states.CallAction_JointTrapVel import CallJointTrap
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
from reconcycle_flexbe_states.switch_controller_service_client import SwitchControllerProxyClient
from reconcycle_flexbe_states.write_to_mongodb import WriteToMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 30 2021
@author: Kubra Karacan, Matija Mavsar, Mihael Simonic
'''
class LeveringActionKalo15HCASM(Behavior):
	'''
	1. Joint move to  levering point
2. Levering action based on Force Action
	'''


	def __init__(self):
		super(LeveringActionKalo15HCASM, self).__init__()
		self.name = 'Levering Action Kalo 1.5 HCA'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1035 y:611, x:507 y:610
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.j_test = [0.973548,0.50085,-0.535146,-1.80296,-0.517136,1.74385,1.56292]
		_state_machine.userdata.j_test_name = 'j_test'
		_state_machine.userdata.db_pose_vice = 'test/pose_vice'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:89
			OperatableStateMachine.add('Load controllers',
										LoadControllerProxyClient(desired_controller='joint_impedance_controller', robot_name=''),
										transitions={'continue': 'Load controllers_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:29 y:177
			OperatableStateMachine.add('Load controllers_2',
										LoadControllerProxyClient(desired_controller='cartesian_impedance_controller_tum', robot_name=''),
										transitions={'continue': 'Switch to joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:647 y:38
			OperatableStateMachine.add('Move to vice',
										CallJointTrap(max_vel=1, max_acl=1, namespace=''),
										transitions={'continue': 'Switch to Cartesian', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_j_test', 'joint_values': 'joint_values'})

			# x:433 y:37
			OperatableStateMachine.add('Read_above_table',
										ReadFromMongo(),
										transitions={'continue': 'Move to vice', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_test_name', 'joints_data': 'mdb_j_test'})

			# x:855 y:486
			OperatableStateMachine.add('Start force action',
										CallForceAction(direction=[0, 1, 0], amplitude=0.02, frequency=3, force=15, rho_min=0.7, namespace=''),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'success': 'success'})

			# x:858 y:54
			OperatableStateMachine.add('Switch to Cartesian',
										SwitchControllerProxyClient(robot_name='', start_controller=['cartesian_impedance_controller_tum'], stop_controller=['joint_impedance_controller'], strictness=2),
										transitions={'continue': 'Start force action', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:20 y:272
			OperatableStateMachine.add('Switch to joint controllers',
										SwitchControllerProxyClient(robot_name='', start_controller=['joint_impedance_controller'], stop_controller=['cartesian_impedance_controller_tum'], strictness=2),
										transitions={'continue': 'write', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:228 y:35
			OperatableStateMachine.add('write',
										WriteToMongo(),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_data': 'j_test', 'entry_name': 'j_test_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
