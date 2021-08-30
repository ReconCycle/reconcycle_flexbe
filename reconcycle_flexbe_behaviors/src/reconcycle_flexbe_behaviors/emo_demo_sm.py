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
from reconcycle_flexbe_states.CallAction_TF_CartLin import CallActionTFCartLin
from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.read_from_mongodb_POSE import ReadFromMongoPOSE
from reconcycle_flexbe_states.write_to_mongodb_pose import WriteToMongoPOSE
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 14 2021
@author: Rok Pahic...
'''
class EMOdemoSM(Behavior):
	'''
	Demo za EMO:
Preberi pozicijo HCA iz kamere
Poberi z Qb hand
Daj v primez
Stisni in zavrti tja in nazaj
Spusti primez 
Poberi z robotm
Odlozi nazaj na zacetek
Ponovi
	'''


	def __init__(self):
		super(EMOdemoSM, self).__init__()
		self.name = 'EMO demo'

		# parameters of this behavior
		self.add_parameter('test', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:413 y:468, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.position1 = [0.1,-0.4,0.50]
		_state_machine.userdata.orentation1 = [2.8,0.2,1.25]
		_state_machine.userdata.pose1_name = "test_pose"
		_state_machine.userdata.grab_pos = [0.7]
		_state_machine.userdata.true = True
		_state_machine.userdata.false = False
		_state_machine.userdata.release_pos = [0.2]
		_state_machine.userdata.robot_pos_grab = [0.1,-0.4,0.45]
		_state_machine.userdata.robot_or_grab = [2.8,0.2,1.25]
		_state_machine.userdata.pose2_name = "grab_position"
		_state_machine.userdata.position2 = [0.1,-0.4,0.40]
		_state_machine.userdata.orentation2 = [2.8,0.2,1.25]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_move_cart_2_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position', 'orentation', 'name'])

		with _sm_move_cart_2_0:
			# x:294 y:42
			OperatableStateMachine.add('test',
										WriteToMongoPOSE(),
										transitions={'continue': 'read pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'position': 'position', 'orientation': 'orentation', 'entry_name': 'name'})

			# x:500 y:377
			OperatableStateMachine.add('read pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'move', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'entry_name': 'name', 'joints_data': 'pose'})

			# x:572 y:81
			OperatableStateMachine.add('move',
										CallActionTFCartLin(namespace="", exe_time=3),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'t2_data': 'pose', 't2_out': 't2_out'})


		# x:30 y:365, x:130 y:365
		_sm_move_cart_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position', 'orentation', 'name'])

		with _sm_move_cart_1:
			# x:294 y:42
			OperatableStateMachine.add('test',
										WriteToMongoPOSE(),
										transitions={'continue': 'read pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'position': 'position', 'orientation': 'orentation', 'entry_name': 'name'})

			# x:500 y:377
			OperatableStateMachine.add('read pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'move', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'entry_name': 'name', 'joints_data': 'pose'})

			# x:572 y:81
			OperatableStateMachine.add('move',
										CallActionTFCartLin(namespace="", exe_time=3),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'t2_data': 'pose', 't2_out': 't2_out'})


		# x:30 y:365, x:130 y:365
		_sm_init_cell_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true'])

		with _sm_init_cell_2:
			# x:126 y:133
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})


		# x:794 y:301, x:223 y:588
		_sm_close_and_rotate_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false'])

		with _sm_close_and_rotate_3:
			# x:115 y:92
			OperatableStateMachine.add('close',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:531 y:307
			OperatableStateMachine.add('open',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:312 y:86
			OperatableStateMachine.add('rotate down',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'wait1_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:520 y:82
			OperatableStateMachine.add('rotate up',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'wait1_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:123 y:210
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'rotate down'},
										autonomy={'done': Autonomy.Off})

			# x:315 y:211
			OperatableStateMachine.add('wait1_2',
										WaitState(wait_time=1),
										transitions={'done': 'rotate up'},
										autonomy={'done': Autonomy.Off})

			# x:564 y:196
			OperatableStateMachine.add('wait1_3',
										WaitState(wait_time=1),
										transitions={'done': 'open'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Init cell',
										_sm_init_cell_2,
										transitions={'finished': 'move cart', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true'})

			# x:364 y:254
			OperatableStateMachine.add('grab',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'Close and rotate', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'grab_pos', 'success': 'success'})

			# x:371 y:39
			OperatableStateMachine.add('move cart',
										_sm_move_cart_1,
										transitions={'finished': 'move cart_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position': 'position1', 'orentation': 'orentation1', 'name': 'pose1_name'})

			# x:373 y:146
			OperatableStateMachine.add('move cart_2',
										_sm_move_cart_2_0,
										transitions={'finished': 'grab', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position': 'position2', 'orentation': 'orentation2', 'name': 'pose2_name'})

			# x:363 y:350
			OperatableStateMachine.add('Close and rotate',
										_sm_close_and_rotate_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
