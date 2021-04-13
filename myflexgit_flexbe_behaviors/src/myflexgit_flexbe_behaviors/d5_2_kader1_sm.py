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
from myflexgit_flexbe_behaviors.breaking_object_sm import BreakingobjectSM
from myflexgit_flexbe_behaviors.change_tool_on_robot_sm import ChangetoolonrobotSM
from myflexgit_flexbe_behaviors.pick_plastic_from_clamp_sm import PickplasticfromclampSM
from myflexgit_flexbe_behaviors.putt_object_in_clamp_sm import PuttobjectinclampSM
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
class D5_2_kader1SM(Behavior):
	'''
	Video for D5_2 until pcb turn
	'''


	def __init__(self):
		super(D5_2_kader1SM, self).__init__()
		self.name = 'D5_2_kader1'

		# parameters of this behavior
		self.add_parameter('tray_service_name', '/clamping_tray')
		self.add_parameter('panda_2_name', 'panda_2')

		# references to used behaviors
		self.add_behavior(BreakingobjectSM, 'Cell runing/Breaking object')
		self.add_behavior(ChangetoolonrobotSM, 'Cell runing/Group/Change tool on robot')
		self.add_behavior(PickplasticfromclampSM, 'Cell runing/Group/Plastic Drop/Pick plastic from clamp')
		self.add_behavior(PuttobjectinclampSM, 'Cell runing/Putt object in clamp')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:741 y:94, x:723 y:338
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.TR = True
		_state_machine.userdata.panda1_init_position = 'panda_1_init'
		_state_machine.userdata.open_hand = [0.1]
		_state_machine.userdata.closed_hand_table = [0.6]
		_state_machine.userdata.closed_hand_clamp = [0.84]
		_state_machine.userdata.FA = False
		_state_machine.userdata.panda1_table_pick = 'panda_1_pick_up'
		_state_machine.userdata.panda1_clamp_pick = 'panda_1_from_clamp'
		_state_machine.userdata.panda1_clamp_release = 'panda_1_drop_down1'
		_state_machine.userdata.panda1_plastic_release = 'panda_1_throw_plastic'
		_state_machine.userdata.panda1_waiting_point = 'holding_point_panda_1'
		_state_machine.userdata.panda2_clamp_hold = 'panda_2_hold'
		_state_machine.userdata.panda2_clamp_break_trj = 'breaking_object_n1'
		_state_machine.userdata.panda2_clamp_retreat_trj = 'trj_umik_2'
		_state_machine.userdata.before_pick_pcb = 'panda_2_beffore_tray'
		_state_machine.userdata.panda_2_init = 'panda_2_beffore_tray'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:797 y:145
		_sm_cell_shotdown_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'])

		with _sm_cell_shotdown_0:
			# x:212 y:121
			OperatableStateMachine.add('Deactivate CLAMP air block',
										ActivateRaspiDigitalOuput(service_name='obr_block2_ON'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_move_over_take_location_2_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_over_take_location_2_1:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'position_name', 'joints_data': 'joints_positions'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_2'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joints_data': 'joints_positions', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_throw_pcb_from_plastic_2 = OperatableStateMachine(outcomes=['continue', 'failed'], input_keys=['FA', 'TR'])

		with _sm_throw_pcb_from_plastic_2:
			# x:314 y:6
			OperatableStateMachine.add('Pull in tray',
										ActivateRaspiDigitalOuput(service_name=self.tray_service_name),
										transitions={'continue': 'Wait to get back ', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:491 y:426
			OperatableStateMachine.add('Push out tray',
										ActivateRaspiDigitalOuput(service_name=self.tray_service_name),
										transitions={'continue': 'Open clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:673 y:32
			OperatableStateMachine.add('Rotate CLAMP down',
										ActivateRaspiDigitalOuput(service_name='/obr_rotate'),
										transitions={'continue': 'Wait to PCB fall', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})

			# x:818 y:260
			OperatableStateMachine.add('Rotate CLAMP up',
										ActivateRaspiDigitalOuput(service_name='/obr_rotate'),
										transitions={'continue': 'Wait to get back _2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:858 y:37
			OperatableStateMachine.add('Wait to PCB fall',
										WaitState(wait_time=3.0),
										transitions={'done': 'Rotate CLAMP up'},
										autonomy={'done': Autonomy.Low})

			# x:497 y:28
			OperatableStateMachine.add('Wait to get back ',
										WaitState(wait_time=2.0),
										transitions={'done': 'Rotate CLAMP down'},
										autonomy={'done': Autonomy.Low})

			# x:694 y:382
			OperatableStateMachine.add('Wait to get back _2',
										WaitState(wait_time=3.0),
										transitions={'done': 'Push out tray'},
										autonomy={'done': Autonomy.Low})

			# x:294 y:491
			OperatableStateMachine.add('Open clamp',
										ActivateRaspiDigitalOuput(service_name='/obr_activate'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_plastic_drop_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'])

		with _sm_plastic_drop_3:
			# x:65 y:119
			OperatableStateMachine.add('Throw PCB from plastic',
										_sm_throw_pcb_from_plastic_2,
										transitions={'continue': 'Pick plastic from clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'FA': 'FA', 'TR': 'TR'})

			# x:243 y:238
			OperatableStateMachine.add('Pick plastic from clamp',
										self.use_behavior(PickplasticfromclampSM, 'Cell runing/Group/Plastic Drop/Pick plastic from clamp',
											default_keys=['clamp_waiting_location_name','closed_hand_clamp','clamp_pick_location_name','clamp_above_location_name'],
											parameters={'max_acl': 1, 'max_vel': 1, 'max_vel_contact': 0.6, 'max_acl_contact': 0.4}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_group_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'], conditions=[
										('finished', [('Change tool on robot', 'finished'), ('Plastic Drop', 'finished')]),
										('failed', [('Change tool on robot', 'failed')]),
										('failed', [('Plastic Drop', 'failed')])
										])

		with _sm_group_4:
			# x:30 y:97
			OperatableStateMachine.add('Change tool on robot',
										self.use_behavior(ChangetoolonrobotSM, 'Cell runing/Group/Change tool on robot',
											default_keys=['tool_drop_location_name','tool_take_location_name','before_drop_location_name','after_take_location_name','open_air_block'],
											parameters={'max_acl': 0.5, 'max_vel': 0.5}),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:252 y:40
			OperatableStateMachine.add('Plastic Drop',
										_sm_plastic_drop_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'FA': 'FA'})


		# x:23 y:588, x:571 y:251
		_sm_cell_runing_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'panda1_init_position', 'open_hand', 'FA', 'closed_hand_table', 'closed_hand_clamp', 'pos_table', 'clamp_release', 'clamp_pick', 'drop_plastic', 'panda1_holding_pos', 'panda2_clamp_hold', 'panda2_clamp_break_trj', 'panda2_clamp_retreat_trj', 'before_pick_pcb'])

		with _sm_cell_runing_5:
			# x:494 y:68
			OperatableStateMachine.add('Putt object in clamp',
										self.use_behavior(PuttobjectinclampSM, 'Cell runing/Putt object in clamp',
											default_keys=['object_table_location_name','clamp_release_location_name','clamp_waiting_location_name','closed_hand_table']),
										transitions={'finished': 'Breaking object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:844 y:454
			OperatableStateMachine.add('Group',
										_sm_group_4,
										transitions={'finished': 'Move over take location_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'FA': 'FA'})

			# x:408 y:475
			OperatableStateMachine.add('Move over take location_2',
										_sm_move_over_take_location_2_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'before_pick_pcb'})

			# x:760 y:182
			OperatableStateMachine.add('Breaking object',
										self.use_behavior(BreakingobjectSM, 'Cell runing/Breaking object',
											default_keys=['trj_name'],
											parameters={'max_vel': 0.7, 'max_acl': 0.7}),
										transitions={'finished': 'Group', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:1175 y:68, x:27 y:668
		_sm_cell_init_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'panda1_init_position', 'open_hand', 'FA', 'panda_2_init'])

		with _sm_cell_init_6:
			# x:40 y:46
			OperatableStateMachine.add('Activate CLAMP air block ',
										ActivateRaspiDigitalOuput(service_name='/obr_block2_ON'),
										transitions={'continue': 'Open clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:594 y:56
			OperatableStateMachine.add('Move Panda 1 to initial',
										CallJointTrap(max_vel=0.3, max_acl=0.3, namespace='/panda_1'),
										transitions={'continue': 'Read initial state_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'init_position', 'joint_values': 'joint_values'})

			# x:785 y:293
			OperatableStateMachine.add('Move Panda 2 to initial_2',
										CallJointTrap(max_vel=0.3, max_acl=0.3, namespace='/panda_2'),
										transitions={'continue': 'Open hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'init_position2', 'joint_values': 'joint_values'})

			# x:217 y:24
			OperatableStateMachine.add('Open clamp',
										ActivateRaspiDigitalOuput(service_name='/obr_activate'),
										transitions={'continue': 'Read initial state', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:867 y:47
			OperatableStateMachine.add('Open hand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_hand', 'success': 'success'})

			# x:390 y:56
			OperatableStateMachine.add('Read initial state',
										ReadFromMongo(),
										transitions={'continue': 'Move Panda 1 to initial', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'panda1_init_position', 'joints_data': 'init_position'})

			# x:575 y:294
			OperatableStateMachine.add('Read initial state_2',
										ReadFromMongo(),
										transitions={'continue': 'Move Panda 2 to initial_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'panda_2_init', 'joints_data': 'init_position2'})



		with _state_machine:
			# x:513 y:35
			OperatableStateMachine.add('Cell init',
										_sm_cell_init_6,
										transitions={'finished': 'Cell runing', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'panda1_init_position': 'panda1_init_position', 'open_hand': 'open_hand', 'FA': 'FA', 'panda_2_init': 'panda_2_init'})

			# x:425 y:359
			OperatableStateMachine.add('Cell runing',
										_sm_cell_runing_5,
										transitions={'finished': 'Cell shotdown', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'panda1_init_position': 'panda1_init_position', 'open_hand': 'open_hand', 'FA': 'FA', 'closed_hand_table': 'closed_hand_table', 'closed_hand_clamp': 'closed_hand_clamp', 'pos_table': 'panda1_table_pick', 'clamp_release': 'panda1_clamp_release', 'clamp_pick': 'panda1_clamp_pick', 'drop_plastic': 'panda1_plastic_release', 'panda1_holding_pos': 'panda1_waiting_point', 'panda2_clamp_hold': 'panda2_clamp_hold', 'panda2_clamp_break_trj': 'panda2_clamp_break_trj', 'panda2_clamp_retreat_trj': 'panda2_clamp_retreat_trj', 'before_pick_pcb': 'before_pick_pcb'})

			# x:77 y:542
			OperatableStateMachine.add('Cell shotdown',
										_sm_cell_shotdown_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'FA': 'FA'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
