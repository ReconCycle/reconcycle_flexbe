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
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.CallAction_JointTrapVel import CallJointTrap
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Mar 26 2021
@author: Rok Pahic
'''
class PickplasticfromclampSM(Behavior):
	'''
	Pick up plastic from clamp and put it in the bin
	'''


	def __init__(self):
		super(PickplasticfromclampSM, self).__init__()
		self.name = 'Pick plastic from clamp'

		# parameters of this behavior
		self.add_parameter('clamp_service_name', '/obr_activate')
		self.add_parameter('max_acl', 1)
		self.add_parameter('namespace', 'panda_1')
		self.add_parameter('max_vel', 1)
		self.add_parameter('max_vel_contact', 0.3)
		self.add_parameter('max_acl_contact', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1121 y:485, x:701 y:512
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['clamp_waiting_location_name', 'closed_hand_clamp', 'clamp_pick_location_name', 'clamp_above_location_name'])
		_state_machine.userdata.TR = True
		_state_machine.userdata.open_hand = [0.1]
		_state_machine.userdata.closed_hand_clamp = [0.65]
		_state_machine.userdata.FA = False
		_state_machine.userdata.clamp_pick_location_name = 'panda_1_from_clamp'
		_state_machine.userdata.clamp_above_location_name = 'panda_1_drop_down1'
		_state_machine.userdata.plastic_bin_location_name = 'panda_1_throw_plastic'
		_state_machine.userdata.clamp_waiting_location_name = 'holding_point_panda_1'
		_state_machine.userdata.clamp_above_2 = 'panda1_safe_above_clamp'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_put_plastic_in_bin_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_put_plastic_in_bin_0:
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


		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_spot_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_spot_1:
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


		# x:30 y:365, x:130 y:365
		_sm_move_over_clamp_2_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_clamp_2_2:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'position_name', 'joints_data': 'joints_positions'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										CallJointTrap(max_vel=self.max_vel_contact, max_acl=self.max_acl_contact, namespace=self.namespace),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joints_data': 'joints_positions', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_over_clamp_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_clamp_3:
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


		# x:30 y:365, x:130 y:365
		_sm_move_in_clamp_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_in_clamp_4:
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
			# x:94 y:57
			OperatableStateMachine.add('Open clamp',
										ActivateRaspiDigitalOuput(service_name=self.clamp_service_name),
										transitions={'continue': 'Release hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'FA', 'success': 'success'})

			# x:459 y:12
			OperatableStateMachine.add('Move in clamp',
										_sm_move_in_clamp_4,
										transitions={'finished': 'Grab object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'clamp_pick_location_name'})

			# x:284 y:40
			OperatableStateMachine.add('Move over clamp',
										_sm_move_over_clamp_3,
										transitions={'finished': 'Move in clamp', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'clamp_above_location_name'})

			# x:713 y:123
			OperatableStateMachine.add('Move over clamp_2',
										_sm_move_over_clamp_2_2,
										transitions={'finished': 'Put plastic in bin', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'clamp_above_2'})

			# x:1145 y:202
			OperatableStateMachine.add('Move to safe spot',
										_sm_move_to_safe_spot_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'clamp_waiting_location_name'})

			# x:748 y:21
			OperatableStateMachine.add('Put plastic in bin',
										_sm_put_plastic_in_bin_0,
										transitions={'finished': 'Release object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'plastic_bin_location_name'})

			# x:175 y:169
			OperatableStateMachine.add('Release hand',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'Move over clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'goal_hand_pos': 'open_hand', 'success': 'success'})

			# x:931 y:28
			OperatableStateMachine.add('Release object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'Move to safe spot', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'goal_hand_pos': 'open_hand', 'success': 'success'})

			# x:527 y:137
			OperatableStateMachine.add('Grab object',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'Move over clamp_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'goal_hand_pos': 'closed_hand_clamp', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
