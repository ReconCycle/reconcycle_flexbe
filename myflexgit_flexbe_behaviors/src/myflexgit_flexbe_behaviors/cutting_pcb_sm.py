#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.wait_state import WaitState
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
class CuttingPCBSM(Behavior):
	'''
	Panda robot pickup the PCB, put it in the cutter, pickup the battery
	'''


	def __init__(self):
		super(CuttingPCBSM, self).__init__()
		self.name = 'Cutting PCB'

		# parameters of this behavior
		self.add_parameter('namespace', 'panda_2')
		self.add_parameter('max_vel', 0.6)
		self.add_parameter('max_acl', 0.6)
		self.add_parameter('cutting_time', 6)
		self.add_parameter('cutter_service', '/cutter_activate')
		self.add_parameter('vacuum_service', '/Panda2Vacuum')
		self.add_parameter('vacuum_time', 2)
		self.add_parameter('max_vel_contact', 0.4)
		self.add_parameter('max_acl_contact', 0.4)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 24 300 
		# 1. move to safe position before pick up|n2. move to pcb and start vacuum|n3. move to safe position before cutter|n4. move pcb in clam and release vacuum|n5. move to safe position before cutter|n6. cutt|n7. pickup battery with vacumm|n8. move over the trash and drop battery

		# O 903 312 
		# This alow to not use cutter in debug proces



	def create(self):
		# x:231 y:553, x:267 y:395
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['PCB_location_name', 'simulate_cutter', 'battery_location_name'])
		_state_machine.userdata.TR = True
		_state_machine.userdata.FA = False
		_state_machine.userdata.PCB_location_name = 'panda_2_pcb_location_2'
		_state_machine.userdata.battery_location_name = 'panda_2_battery_pick'
		_state_machine.userdata.tray_safe_position_name = 'panda_2_beffore_tray'
		_state_machine.userdata.cutter_safe_position_name = 'panda_2_beffore_cutter'
		_state_machine.userdata.cutter_drop_position_name = 'panda_2_cutter4'
		_state_machine.userdata.battery_drop_position_name = 'panda_2_drop_battery2'
		_state_machine.userdata.simulate_cutter = True
		_state_machine.userdata.above_tray = 'panda_2_pcb_up'
		_state_machine.userdata.above_cutter = 'panda_2_cutter_up'
		_state_machine.userdata.in_cutter = 'panda_2_in_cutter'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_rise__robot_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_rise__robot_0:
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
		_sm_putt_pcb_in_cutter_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_putt_pcb_in_cutter_1:
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
		_sm_pick_up_battery_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_pick_up_battery_2:
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
		_sm_pick_up_pcb_location_2_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_pick_up_pcb_location_2_3:
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
		_sm_pick_up_pcb_location_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_pick_up_pcb_location_4:
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
		_sm_move_to_safe_position_before_cutter_2_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_position_before_cutter_2_5:
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
		_sm_move_to_safe_position_before_cutter_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_position_before_cutter_6:
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


		# x:842 y:39, x:130 y:365
		_sm_move_to_safe_location_before_pick_up_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_location_before_pick_up_7:
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
		_sm_drop_location_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_drop_location_8:
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
		_sm_drop_battery_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_drop_battery_9:
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
			# x:72 y:28
			OperatableStateMachine.add('Move to safe location before pick up',
										_sm_move_to_safe_location_before_pick_up_7,
										transitions={'finished': 'Pick up PCB location', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'tray_safe_position_name'})

			# x:401 y:105
			OperatableStateMachine.add('Activate vacuum',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:1089 y:613
			OperatableStateMachine.add('Activate vacuum_2',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:932 y:427
			OperatableStateMachine.add('Cutting timer',
										WaitState(wait_time=self.cutting_time),
										transitions={'done': 'Deactivate cutter'},
										autonomy={'done': Autonomy.Low})

			# x:906 y:502
			OperatableStateMachine.add('Deactivate cutter',
										ActivateRaspiDigitalOuput(service_name=self.cutter_service),
										transitions={'continue': 'Pick up battery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'FA', 'success': 'success'})

			# x:930 y:92
			OperatableStateMachine.add('Deactivate vacuum ',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer 2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:481 y:603
			OperatableStateMachine.add('Deactivate vacuum_3',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:681 y:605
			OperatableStateMachine.add('Drop battery',
										_sm_drop_battery_9,
										transitions={'finished': 'Deactivate vacuum_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'battery_drop_position_name'})

			# x:1100 y:12
			OperatableStateMachine.add('Drop location',
										_sm_drop_location_8,
										transitions={'finished': 'Putt PCB in cutter', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'in_cutter'})

			# x:803 y:22
			OperatableStateMachine.add('Move to safe position before cutter',
										_sm_move_to_safe_position_before_cutter_6,
										transitions={'finished': 'Drop location', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'cutter_safe_position_name'})

			# x:1057 y:239
			OperatableStateMachine.add('Move to safe position before cutter_2',
										_sm_move_to_safe_position_before_cutter_2_5,
										transitions={'finished': 'Simulate cutter 2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'cutter_safe_position_name'})

			# x:361 y:27
			OperatableStateMachine.add('Pick up PCB location',
										_sm_pick_up_pcb_location_4,
										transitions={'finished': 'Activate vacuum', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'PCB_location_name'})

			# x:579 y:23
			OperatableStateMachine.add('Pick up PCB location 2',
										_sm_pick_up_pcb_location_2_3,
										transitions={'finished': 'Move to safe position before cutter', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'above_tray'})

			# x:1124 y:487
			OperatableStateMachine.add('Pick up battery',
										_sm_pick_up_battery_2,
										transitions={'finished': 'Activate vacuum_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'battery_location_name'})

			# x:1106 y:87
			OperatableStateMachine.add('Putt PCB in cutter',
										_sm_putt_pcb_in_cutter_1,
										transitions={'finished': 'Deactivate vacuum ', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'cutter_drop_position_name'})

			# x:1107 y:167
			OperatableStateMachine.add('Rise  robot',
										_sm_rise__robot_0,
										transitions={'finished': 'Move to safe position before cutter_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'in_cutter'})

			# x:1110 y:342
			OperatableStateMachine.add('Simulate cutter 2',
										CheckConditionState(predicate=lambda x: x == True),
										transitions={'true': 'Pick up battery', 'false': 'Activate cutter'},
										autonomy={'true': Autonomy.Full, 'false': Autonomy.Full},
										remapping={'input_value': 'simulate_cutter'})

			# x:600 y:118
			OperatableStateMachine.add('Vacuum timer',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'Pick up PCB location 2'},
										autonomy={'done': Autonomy.Low})

			# x:940 y:168
			OperatableStateMachine.add('Vacuum timer 2',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'Rise  robot'},
										autonomy={'done': Autonomy.High})

			# x:894 y:613
			OperatableStateMachine.add('Vacuum timer_2',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'Drop battery'},
										autonomy={'done': Autonomy.Low})

			# x:278 y:602
			OperatableStateMachine.add('Vacuum timer_3',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low})

			# x:886 y:348
			OperatableStateMachine.add('Activate cutter',
										ActivateRaspiDigitalOuput(service_name=self.cutter_service),
										transitions={'continue': 'Cutting timer', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
