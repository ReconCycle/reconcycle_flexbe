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
from myflexgit_flexbe_states.MoveSoftHand import MoveSoftHand
from myflexgit_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from myflexgit_flexbe_states.call_joint_trap_vel_action_server import CallJointTrap
from myflexgit_flexbe_states.read_from_mongodb import ReadFromMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Mar 26 2021
@author: Rok Pahic
'''
class PuttobjectinclampSM(Behavior):
	'''
	Pick up heat alocator from table and put in clamp
	'''


	def __init__(self):
		super(PuttobjectinclampSM, self).__init__()
		self.name = 'Putt object in clamp'

		# parameters of this behavior
		self.add_parameter('clamp_service_name', '/obr_activate')
		self.add_parameter('max_acl', 1)
		self.add_parameter('namespace', 'panda_1')
		self.add_parameter('max_vel', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1121 y:485, x:321 y:373
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['object_table_location_name', 'clamp_release_location_name', 'clamp_waiting_location_name', 'closed_hand_table'])
		_state_machine.userdata.TR = True
		_state_machine.userdata.open_hand = [0.1]
		_state_machine.userdata.closed_hand_table = [0.65]
		_state_machine.userdata.FA = False
		_state_machine.userdata.object_table_location_name = 'panda_1_pick_up'
		_state_machine.userdata.clamp_high_location_name = 'panda1_high_above_clamp'
		_state_machine.userdata.clamp_waiting_location_name = 'holding_point_panda_1'
		_state_machine.userdata.clamp_release_location_name = 'panda1_drop_down3'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_put_object_in_clamp_2_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_put_object_in_clamp_2_0:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'position_name', 'joints_data': 'joints_positions'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace=self.namespace),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.High, 'failed': Autonomy.High},
										remapping={'joints_data': 'joints_positions', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_pick_object_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_pick_object_1:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'position_name', 'joints_data': 'joints_positions'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace=self.namespace),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.High, 'failed': Autonomy.High},
										remapping={'joints_data': 'joints_positions', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_spot_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_spot_2:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'position_name', 'joints_data': 'joints_positions'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace=self.namespace),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joints_data': 'joints_positions', 'joint_values': 'joint_values'})



		with _state_machine:
			# x:43 y:209
			OperatableStateMachine.add('Close clamp_2_2',
										ActivateRaspiDigitalOuput(service_name='/obr_block2_ON'),
										transitions={'continue': 'Close clamp_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.High, 'failed': Autonomy.High},
										remapping={'value': 'TR', 'success': 'success'})

			# x:43 y:147
			OperatableStateMachine.add('Close clamp_2',
										ActivateRaspiDigitalOuput(service_name=self.clamp_service_name),
										transitions={'continue': 'Release object_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.High, 'failed': Autonomy.High},
										remapping={'value': 'FA', 'success': 'success'})

			# x:517 y:20
			OperatableStateMachine.add('Grab object',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'Put object in clamp_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'goal_hand_pos': 'closed_hand_table', 'success': 'success'})

			# x:1145 y:202
			OperatableStateMachine.add('Move to safe spot',
										_sm_move_to_safe_spot_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'clamp_waiting_location_name'})

			# x:340 y:28
			OperatableStateMachine.add('Pick object',
										_sm_pick_object_1,
										transitions={'finished': 'Grab object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'object_table_location_name'})

			# x:690 y:97
			OperatableStateMachine.add('Put object in clamp_2',
										_sm_put_object_in_clamp_2_0,
										transitions={'finished': 'Release object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'clamp_release_location_name'})

			# x:931 y:28
			OperatableStateMachine.add('Release object',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'wait to drop', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'goal_hand_pos': 'open_hand', 'success': 'success'})

			# x:114 y:44
			OperatableStateMachine.add('Release object_2',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'Pick object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'goal_hand_pos': 'open_hand', 'success': 'success'})

			# x:1134 y:25
			OperatableStateMachine.add('wait to drop',
										WaitState(wait_time=1),
										transitions={'done': 'Close clamp'},
										autonomy={'done': Autonomy.Low})

			# x:1163 y:122
			OperatableStateMachine.add('Close clamp',
										ActivateRaspiDigitalOuput(service_name=self.clamp_service_name),
										transitions={'continue': 'Move to safe spot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.High, 'failed': Autonomy.High},
										remapping={'value': 'TR', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
