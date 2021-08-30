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
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.call_cart_rel_action_server import CallCartRel
from reconcycle_flexbe_states.call_joint_trap_vel_action_server import CallJointTrap
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Mar 26 2021
@author: Rok Pahic
'''
class ChangetoolonrobotSM(Behavior):
	'''
	Pick up heat alocator from table and put in clamp
	'''


	def __init__(self):
		super(ChangetoolonrobotSM, self).__init__()
		self.name = 'Change tool on robot'

		# parameters of this behavior
		self.add_parameter('max_acl', 0.3)
		self.add_parameter('namespace', 'panda_2')
		self.add_parameter('max_vel', 0.3)
		self.add_parameter('air_block_service_name', '/Panda2BlockON')
		self.add_parameter('tool_unlock_service_name', '/Panda2ToolUnlock')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:84 y:259, x:723 y:338
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['tool_drop_location_name', 'tool_take_location_name', 'before_drop_location_name', 'after_take_location_name', 'open_air_block'])
		_state_machine.userdata.TR = True
		_state_machine.userdata.FA = False
		_state_machine.userdata.tool_drop_location_name = 'panda2_tool1_location1'
		_state_machine.userdata.tool_take_location_name = 'panda2_tool2_location1'
		_state_machine.userdata.before_drop_location_name = 'panda2_tool1_above'
		_state_machine.userdata.after_take_location_name = 'panda2_tool2_above'
		_state_machine.userdata.open_air_block = False
		_state_machine.userdata.tool_drop_aim = 'panda2_tool1_aim1'
		_state_machine.userdata.tool_take_aim = 'panda2_tool2_aim1'
		_state_machine.userdata.trans_value = [0.0, 0.0, -0.008]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_move_to_take__2_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_take__2_0:
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
		_sm_move_to_take__1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_take__1:
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
		_sm_move_to_drop_2_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_drop_2_2:
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
		_sm_move_to_drop_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_drop_3:
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
		_sm_move_over_take_location_2_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_take_location_2_4:
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
		_sm_move_over_take_location_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_take_location_5:
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
		_sm_move_over_drop_location_2_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_drop_location_2_6:
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
		_sm_move_over_drop_location_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_drop_location_7:
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
			# x:53 y:60
			OperatableStateMachine.add('Move over drop location',
										_sm_move_over_drop_location_7,
										transitions={'finished': 'Move to drop_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'before_drop_location_name'})

			# x:533 y:546
			OperatableStateMachine.add('Lock tool',
										ActivateRaspiDigitalOuput(service_name=self.tool_unlock_service_name),
										transitions={'continue': 'Wait to lock_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'FA', 'success': 'success'})

			# x:987 y:256
			OperatableStateMachine.add('Move over drop location_2',
										_sm_move_over_drop_location_2_6,
										transitions={'finished': 'Move over take location', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'before_drop_location_name'})

			# x:1126 y:530
			OperatableStateMachine.add('Move over take location',
										_sm_move_over_take_location_5,
										transitions={'finished': 'Move to take _2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'after_take_location_name'})

			# x:68 y:530
			OperatableStateMachine.add('Move over take location_2',
										_sm_move_over_take_location_2_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'after_take_location_name'})

			# x:513 y:41
			OperatableStateMachine.add('Move to drop',
										_sm_move_to_drop_3,
										transitions={'finished': 'Close air block', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'tool_drop_location_name'})

			# x:346 y:19
			OperatableStateMachine.add('Move to drop_2',
										_sm_move_to_drop_2_2,
										transitions={'finished': 'Move to drop', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'tool_drop_aim'})

			# x:754 y:633
			OperatableStateMachine.add('Move to take ',
										_sm_move_to_take__1,
										transitions={'finished': 'Press to tool', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'tool_take_location_name'})

			# x:925 y:533
			OperatableStateMachine.add('Move to take _2',
										_sm_move_to_take__2_0,
										transitions={'finished': 'Move to take ', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'tool_take_aim'})

			# x:299 y:442
			OperatableStateMachine.add('Open air block',
										ActivateRaspiDigitalOuput(service_name=self.air_block_service_name),
										transitions={'continue': 'Move over take location_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'open_air_block', 'success': 'success'})

			# x:547 y:653
			OperatableStateMachine.add('Press to tool',
										CallCartRel(exe_time=3, namespace='panda_2'),
										transitions={'continue': 'Lock tool', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'trans_value': 'trans_value', 'success': 'success'})

			# x:900 y:44
			OperatableStateMachine.add('Unlock tool',
										ActivateRaspiDigitalOuput(service_name=self.tool_unlock_service_name),
										transitions={'continue': 'Wait to lock', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})

			# x:945 y:133
			OperatableStateMachine.add('Wait to lock',
										WaitState(wait_time=1),
										transitions={'done': 'Move over drop location_2'},
										autonomy={'done': Autonomy.Off})

			# x:431 y:504
			OperatableStateMachine.add('Wait to lock_2',
										WaitState(wait_time=2),
										transitions={'done': 'Open air block'},
										autonomy={'done': Autonomy.Low})

			# x:676 y:42
			OperatableStateMachine.add('Close air block',
										ActivateRaspiDigitalOuput(service_name=self.air_block_service_name),
										transitions={'continue': 'Unlock tool', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'FA', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
