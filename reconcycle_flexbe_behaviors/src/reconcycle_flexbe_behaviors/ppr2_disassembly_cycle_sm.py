#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from rbs_flexbe_states.ActionPredictor_d import ActionPredictor_d
from rbs_flexbe_states.concurrent_battery_and_hca_frame import Concurrent_battery_and_hca_frame
from rbs_flexbe_states.inst_robotblockset import Instantiate_robotblockset
from rbs_flexbe_states.levering import Levering
from rbs_flexbe_states.pcb_vise_to_cutter import PCB_vise_to_cutter
from rbs_flexbe_states.pick_up_hca import Pick_up_HCA
from rbs_flexbe_states.pinpush import Pinpush
from rbs_flexbe_states.robot_go_to_init import Robot_go_to_init
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 14 2022
@author: Boris Kuster
'''
class PPR2DisassemblyCycleSM(Behavior):
	'''
	Behavior for disassembly of both Qundis and Kalo 1.5 HCA types
	'''


	def __init__(self):
		super(PPR2DisassemblyCycleSM, self).__init__()
		self.name = 'PPR2 Disassembly Cycle'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		cycle_speed = 'fast'
		activate_cutter = False
		# x:1261 y:876, x:1478 y:1
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.adaptive_levering = False
		_state_machine.userdata.levering_direction_deg = 90
		_state_machine.userdata.dummy = True
		_state_machine.userdata.hca_type = None
		_state_machine.userdata.has_pin = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:29
			OperatableStateMachine.add('inst_robots',
										Instantiate_robotblockset(cycle_speed=cycle_speed),
										transitions={'continue': 'Move robots to init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'p1': 'p1', 'p2': 'p2', 'cy': 'cy'})

			# x:1246 y:720
			OperatableStateMachine.add('Battery_and_hca_frame_pickup',
										Concurrent_battery_and_hca_frame(),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cy': 'cy', 'dummy': 'dummy'})

			# x:199 y:29
			OperatableStateMachine.add('Move robots to init',
										Robot_go_to_init(),
										transitions={'continue': 'ActionPredictor', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cy': 'cy', 'p1': 'p1', 'p2': 'p2'})

			# x:1301 y:73
			OperatableStateMachine.add('P1_pick_up_HCA',
										Pick_up_HCA(init_safe=False, double_tap_vise=True, return_to_via=True),
										transitions={'kalo': 'ActionPredictor', 'qundis-pin': 'ActionPredictor', 'qundis-no_pin': 'ActionPredictor', 'failed': 'failed'},
										autonomy={'kalo': Autonomy.Off, 'qundis-pin': Autonomy.Off, 'qundis-no_pin': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cy': 'cy', 'dummy': 'dummy', 'hca_type': 'hca_type', 'has_pin': 'has_pin'})

			# x:1279 y:486
			OperatableStateMachine.add('P2_PCB_to_cutter',
										PCB_vise_to_cutter(init_safe=True, activate_cutter=activate_cutter, move_to_realsense_pos=True),
										transitions={'continue': 'Battery_and_hca_frame_pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cy': 'cy', 'hca_type': 'hca_type', 'dummy': 'dummy'})

			# x:1321 y:313
			OperatableStateMachine.add('P2_levering',
										Levering(),
										transitions={'continue': 'ActionPredictor', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cy': 'cy', 'hca_type': 'hca_type', 'levering_direction_deg': 'levering_direction_deg', 'adaptive_levering': 'adaptive_levering', 'dummy': 'dummy', 'adaptive_levering_dT_1': 'adaptive_levering_dT_1', 'adaptive_levering_dT_2': 'adaptive_levering_dT_2'})

			# x:1308 y:175
			OperatableStateMachine.add('P2_pinpush',
										Pinpush(init_safe=True, rotate_vise=False, force_pinpush=False),
										transitions={'continue': 'ActionPredictor', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'cy': 'cy', 'hca_type': 'hca_type', 'has_pin': 'has_pin', 'dummy': 'dummy'})

			# x:553 y:518
			OperatableStateMachine.add('ActionPredictor',
										ActionPredictor_d(),
										transitions={'hca_to_vise': 'P1_pick_up_HCA', 'pinpush': 'P2_pinpush', 'levering': 'P2_levering', 'pcb_to_cutter': 'P2_PCB_to_cutter', 'cutting': 'P2_PCB_to_cutter', 'pick_up_battery': 'Battery_and_hca_frame_pickup', 'hca_frame_to_bin': 'Battery_and_hca_frame_pickup', 'failed': 'failed'},
										autonomy={'hca_to_vise': Autonomy.Off, 'pinpush': Autonomy.Off, 'levering': Autonomy.Off, 'pcb_to_cutter': Autonomy.Off, 'cutting': Autonomy.Off, 'pick_up_battery': Autonomy.Off, 'hca_frame_to_bin': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
