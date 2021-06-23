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
from myflexgit_flexbe_behaviors.cutting_pcb_sm import CuttingPCBSM
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
class D5_2SM(Behavior):
	'''
	Video for D5_2, paraller atempt
	'''


	def __init__(self):
		super(D5_2SM, self).__init__()
		self.name = 'D5_2'

		# parameters of this behavior
		self.add_parameter('tray_service_name', '/clamping_tray')

		# references to used behaviors
		self.add_behavior(BreakingobjectSM, 'Cell runing/Breaking object')
		self.add_behavior(CuttingPCBSM, 'Cell runing/Paralel serving cutter and clamp/Cutting PCB')
		self.add_behavior(PuttobjectinclampSM, 'Cell runing/Paralel serving cutter and clamp/Putt object in clamp')
		self.add_behavior(ChangetoolonrobotSM, 'Cell runing/Paralel tool change and plastic drop/Change tool on robot')
		self.add_behavior(PickplasticfromclampSM, 'Cell runing/Paralel tool change and plastic drop/Plastic hausing drop/Dispose plastic from clamp')
		self.add_behavior(PuttobjectinclampSM, 'Cell runing/Putt object in clamp')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:93 y:385, x:297 y:182
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


		# x:129 y:357, x:411 y:231
		_sm_drop_pcb_from_plastic_1 = OperatableStateMachine(outcomes=['continue', 'failed'], input_keys=['FA', 'TR'])

		with _sm_drop_pcb_from_plastic_1:
			# x:148 y:37
			OperatableStateMachine.add('Pull in tray',
										ActivateRaspiDigitalOuput(service_name=self.tray_service_name),
										transitions={'continue': 'Wait to get back ', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:538 y:347
			OperatableStateMachine.add('Push out tray',
										ActivateRaspiDigitalOuput(service_name=self.tray_service_name),
										transitions={'continue': 'Open clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:508 y:37
			OperatableStateMachine.add('Rotate CLAMP down',
										ActivateRaspiDigitalOuput(service_name='/obr_rotate'),
										transitions={'continue': 'Wait to PCB fall', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})

			# x:703 y:183
			OperatableStateMachine.add('Rotate CLAMP up',
										ActivateRaspiDigitalOuput(service_name='/obr_rotate'),
										transitions={'continue': 'Wait to get back _2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:717 y:38
			OperatableStateMachine.add('Wait to PCB fall',
										WaitState(wait_time=3.0),
										transitions={'done': 'Rotate CLAMP up'},
										autonomy={'done': Autonomy.Low})

			# x:359 y:38
			OperatableStateMachine.add('Wait to get back ',
										WaitState(wait_time=3.0),
										transitions={'done': 'Rotate CLAMP down'},
										autonomy={'done': Autonomy.Low})

			# x:734 y:321
			OperatableStateMachine.add('Wait to get back _2',
										WaitState(wait_time=3.0),
										transitions={'done': 'Push out tray'},
										autonomy={'done': Autonomy.Low})

			# x:277 y:352
			OperatableStateMachine.add('Open clamp',
										ActivateRaspiDigitalOuput(service_name='/obr_activate'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})


		# x:537 y:30, x:408 y:164
		_sm_plastic_hausing_drop_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'])

		with _sm_plastic_hausing_drop_2:
			# x:53 y:26
			OperatableStateMachine.add('Drop PCB from plastic',
										_sm_drop_pcb_from_plastic_1,
										transitions={'continue': 'Dispose plastic from clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'FA': 'FA', 'TR': 'TR'})

			# x:291 y:27
			OperatableStateMachine.add('Dispose plastic from clamp',
										self.use_behavior(PickplasticfromclampSM, 'Cell runing/Paralel tool change and plastic drop/Plastic hausing drop/Dispose plastic from clamp',
											default_keys=['clamp_waiting_location_name','closed_hand_clamp','clamp_pick_location_name','clamp_above_location_name']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:357 y:128, x:355 y:192, x:361 y:51, x:202 y:621, x:326 y:614
		_sm_paralel_tool_change_and_plastic_drop_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'], conditions=[
										('finished', [('Plastic hausing drop', 'finished'), ('Change tool on robot', 'finished')]),
										('failed', [('Plastic hausing drop', 'failed')]),
										('failed', [('Change tool on robot', 'failed')])
										])

		with _sm_paralel_tool_change_and_plastic_drop_3:
			# x:73 y:39
			OperatableStateMachine.add('Change tool on robot',
										self.use_behavior(ChangetoolonrobotSM, 'Cell runing/Paralel tool change and plastic drop/Change tool on robot',
											default_keys=['tool_drop_location_name','tool_take_location_name','before_drop_location_name','after_take_location_name','open_air_block']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:76 y:131
			OperatableStateMachine.add('Plastic hausing drop',
										_sm_plastic_hausing_drop_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'FA': 'FA'})


		# x:398 y:110, x:413 y:35, x:411 y:199, x:330 y:365, x:430 y:365
		_sm_paralel_serving_cutter_and_clamp_4 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('Cutting PCB', 'finished'), ('Putt object in clamp', 'finished')]),
										('failed', [('Cutting PCB', 'failed')]),
										('failed', [('Putt object in clamp', 'failed')])
										])

		with _sm_paralel_serving_cutter_and_clamp_4:
			# x:89 y:31
			OperatableStateMachine.add('Cutting PCB',
										self.use_behavior(CuttingPCBSM, 'Cell runing/Paralel serving cutter and clamp/Cutting PCB',
											default_keys=['PCB_location_name','simulate_cutter','battery_location_name']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:67 y:137
			OperatableStateMachine.add('Putt object in clamp',
										self.use_behavior(PuttobjectinclampSM, 'Cell runing/Paralel serving cutter and clamp/Putt object in clamp',
											default_keys=['object_table_location_name','clamp_release_location_name','clamp_waiting_location_name','closed_hand_table']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:35 y:296, x:15 y:186
		_sm_cell_runing_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'panda1_init_position', 'open_hand', 'FA', 'closed_hand_table', 'closed_hand_clamp', 'pos_table', 'clamp_release', 'clamp_pick', 'drop_plastic', 'panda1_holding_pos', 'panda2_clamp_hold', 'panda2_clamp_break_trj', 'panda2_clamp_retreat_trj'])

		with _sm_cell_runing_5:
			# x:77 y:30
			OperatableStateMachine.add('Putt object in clamp',
										self.use_behavior(PuttobjectinclampSM, 'Cell runing/Putt object in clamp',
											default_keys=['object_table_location_name','clamp_release_location_name','clamp_waiting_location_name','closed_hand_table']),
										transitions={'finished': 'Breaking object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:306 y:256
			OperatableStateMachine.add('Paralel serving cutter and clamp',
										_sm_paralel_serving_cutter_and_clamp_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:307 y:161
			OperatableStateMachine.add('Paralel tool change and plastic drop',
										_sm_paralel_tool_change_and_plastic_drop_3,
										transitions={'finished': 'Paralel serving cutter and clamp', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'FA': 'FA'})

			# x:335 y:31
			OperatableStateMachine.add('Breaking object',
										self.use_behavior(BreakingobjectSM, 'Cell runing/Breaking object',
											default_keys=['trj_name']),
										transitions={'finished': 'Paralel tool change and plastic drop', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:1175 y:68, x:27 y:668
		_sm_cell_init_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'panda1_init_position', 'open_hand', 'FA'])

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
										transitions={'continue': 'Open hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'init_position', 'joint_values': 'joint_values'})

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



		with _state_machine:
			# x:65 y:34
			OperatableStateMachine.add('Cell init',
										_sm_cell_init_6,
										transitions={'finished': 'Cell runing', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'panda1_init_position': 'panda1_init_position', 'open_hand': 'open_hand', 'FA': 'FA'})

			# x:67 y:146
			OperatableStateMachine.add('Cell runing',
										_sm_cell_runing_5,
										transitions={'finished': 'Cell shotdown', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'panda1_init_position': 'panda1_init_position', 'open_hand': 'open_hand', 'FA': 'FA', 'closed_hand_table': 'closed_hand_table', 'closed_hand_clamp': 'closed_hand_clamp', 'pos_table': 'panda1_table_pick', 'clamp_release': 'panda1_clamp_release', 'clamp_pick': 'panda1_clamp_pick', 'drop_plastic': 'panda1_plastic_release', 'panda1_holding_pos': 'panda1_waiting_point', 'panda2_clamp_hold': 'panda2_clamp_hold', 'panda2_clamp_break_trj': 'panda2_clamp_break_trj', 'panda2_clamp_retreat_trj': 'panda2_clamp_retreat_trj'})

			# x:66 y:265
			OperatableStateMachine.add('Cell shotdown',
										_sm_cell_shotdown_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'TR', 'FA': 'FA'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
