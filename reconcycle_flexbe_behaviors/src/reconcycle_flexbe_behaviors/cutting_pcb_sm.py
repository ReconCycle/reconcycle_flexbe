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
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.call_joint_trap_vel_action_server import CallJointTrap
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
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

		# O 997 354 
		# This alow to not use cutter in debug proces



	def create(self):
		# x:21 y:679, x:267 y:395
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['PCB_location_name', 'simulate_cutter', 'battery_location_name'])
		_state_machine.userdata.TR = True
		_state_machine.userdata.FA = False
		_state_machine.userdata.PCB_location_name = 'panda_2_pcb_location'
		_state_machine.userdata.battery_location_name = 'panda_2_battery_pick'
		_state_machine.userdata.tray_safe_position_name = 'panda_2_beffore_tray'
		_state_machine.userdata.cutter_safe_position_name = 'panda_2_battery_aim'
		_state_machine.userdata.cutter_drop_position_name = 'panda_2_cutter2'
		_state_machine.userdata.battery_drop_position_name = 'panda_2_drop_battery2'
		_state_machine.userdata.simulate_cutter = True

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_putt_pcb_in_cutter_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_putt_pcb_in_cutter_0:
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
		_sm_pick_up_battery_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_pick_up_battery_1:
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
		_sm_pick_up_pcb_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_pick_up_pcb_2:
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
		_sm_move_to_safe_position_before_cutter_2_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_position_before_cutter_2_3:
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
		_sm_move_to_safe_position_before_cutter_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_position_before_cutter_4:
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
		_sm_move_to_safe_location_before_pick_up_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_location_before_pick_up_5:
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
		_sm_drop_battery_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_drop_battery_6:
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
										_sm_move_to_safe_location_before_pick_up_5,
										transitions={'finished': 'Pick up PCB', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'tray_safe_position_name'})

			# x:526 y:34
			OperatableStateMachine.add('Activate vacuum',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:907 y:627
			OperatableStateMachine.add('Activate vacuum_2',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:988 y:443
			OperatableStateMachine.add('Cutting timer',
										WaitState(wait_time=self.cutting_time),
										transitions={'done': 'Deactivate cutter'},
										autonomy={'done': Autonomy.Low})

			# x:968 y:515
			OperatableStateMachine.add('Deactivate cutter',
										ActivateRaspiDigitalOuput(service_name=self.cutter_service),
										transitions={'continue': 'Pick up battery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'FA', 'success': 'success'})

			# x:316 y:645
			OperatableStateMachine.add('Deactivate vacuum_3',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:1113 y:186
			OperatableStateMachine.add('Deactivate vacuum_3_2',
										ActivateRaspiDigitalOuput(service_name=self.vacuum_service),
										transitions={'continue': 'Vacuum timer_3_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:562 y:646
			OperatableStateMachine.add('Drop battery',
										_sm_drop_battery_6,
										transitions={'finished': 'Deactivate vacuum_3', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'battery_drop_position_name'})

			# x:907 y:35
			OperatableStateMachine.add('Move to safe position before cutter',
										_sm_move_to_safe_position_before_cutter_4,
										transitions={'finished': 'Putt PCB in cutter', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'cutter_safe_position_name'})

			# x:820 y:281
			OperatableStateMachine.add('Move to safe position before cutter_2',
										_sm_move_to_safe_position_before_cutter_2_3,
										transitions={'finished': 'Simulate cutter 2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'cutter_safe_position_name'})

			# x:374 y:36
			OperatableStateMachine.add('Pick up PCB',
										_sm_pick_up_pcb_2,
										transitions={'finished': 'Activate vacuum', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'PCB_location_name'})

			# x:1115 y:629
			OperatableStateMachine.add('Pick up battery',
										_sm_pick_up_battery_1,
										transitions={'finished': 'Activate vacuum_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'battery_location_name'})

			# x:1160 y:36
			OperatableStateMachine.add('Putt PCB in cutter',
										_sm_putt_pcb_in_cutter_0,
										transitions={'finished': 'Deactivate vacuum_3_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'cutter_drop_position_name'})

			# x:1149 y:388
			OperatableStateMachine.add('Simulate cutter 2',
										CheckConditionState(predicate=lambda x: x == True),
										transitions={'true': 'Pick up battery', 'false': 'Activate cutter'},
										autonomy={'true': Autonomy.High, 'false': Autonomy.High},
										remapping={'input_value': 'simulate_cutter'})

			# x:741 y:37
			OperatableStateMachine.add('Vacuum timer',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'Move to safe position before cutter'},
										autonomy={'done': Autonomy.Low})

			# x:740 y:618
			OperatableStateMachine.add('Vacuum timer_2',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'Drop battery'},
										autonomy={'done': Autonomy.Low})

			# x:139 y:622
			OperatableStateMachine.add('Vacuum timer_3',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low})

			# x:1129 y:279
			OperatableStateMachine.add('Vacuum timer_3_2',
										WaitState(wait_time=self.vacuum_time),
										transitions={'done': 'Move to safe position before cutter_2'},
										autonomy={'done': Autonomy.High})

			# x:899 y:381
			OperatableStateMachine.add('Activate cutter',
										ActivateRaspiDigitalOuput(service_name=self.cutter_service),
										transitions={'continue': 'Cutting timer', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
