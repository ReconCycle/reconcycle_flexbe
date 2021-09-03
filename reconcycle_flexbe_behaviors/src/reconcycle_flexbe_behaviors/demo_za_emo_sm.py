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
from reconcycle_flexbe_states.CallAction_JointTrapVel_mult import CallJointTrap
from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.read_from_mongodb_mult import ReadFromMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 14 2021
@author: Rok Pahic, Boris Kuster, Mihael Simonic, Matija Mavsar
'''
class DemozaEMOSM(Behavior):
	'''
	Potrebna oprema:

bambus-1.local: roslaunch controllers 
raspi-panda1-block.local: docker
raspi-clamp-block.local: docker


Opis:
- Preberi pozicijo HCA iz kamere
- Poberi z Qb hand
- Daj v primez
- Stisni in zavrti tja in nazaj
- Spusti primez 
- Poberi z robotm
- Odlozi nazaj na zacetek
- Ponovi
	'''


	def __init__(self):
		super(DemozaEMOSM, self).__init__()
		self.name = 'Demo za EMO'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:459 y:549, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grab_pos = [0.7]
		_state_machine.userdata.true = True
		_state_machine.userdata.false = False
		_state_machine.userdata.release_pos = [0.2]
		_state_machine.userdata.init_pose = "emo/j_init_pose"
		_state_machine.userdata.pickup_pose = "emo/j_pickup_pose"
		_state_machine.userdata.slider_pose_above = "emo/j_slider_pose_above"
		_state_machine.userdata.slider_pose_near = "emo/j_slider_pose_near"
		_state_machine.userdata.holder_pickup_pose = "emo/j_holder_pickup_pose"
		_state_machine.userdata.holder_up_pose = "emo/j_holder_up_pose"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:164 y:172
		_sm_pickup_from_holder_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grab_pos', 'holder_pickup_pose', 'holder_up_pose'])

		with _sm_pickup_from_holder_0:
			# x:99 y:42
			OperatableStateMachine.add('Read holder pickup',
										ReadFromMongo(),
										transitions={'continue': 'Read holder up', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name_array': 'holder_pickup_pose', 'joints_data': 'joints_data'})

			# x:297 y:273
			OperatableStateMachine.add('Move holder up',
										CallJointTrap(max_vel=0.5, max_acl=0.5, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:302 y:40
			OperatableStateMachine.add('Read holder up',
										ReadFromMongo(),
										transitions={'continue': 'Move holder pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name_array': 'holder_up_pose', 'joints_data': 'joints_data'})

			# x:298 y:158
			OperatableStateMachine.add('Move holder pickup',
										CallJointTrap(max_vel=0.5, max_acl=0.5, namespace="panda_1"),
										transitions={'continue': 'Move holder up', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})


		# x:30 y:365, x:181 y:175
		_sm_move_to_slider_and_drop_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['release_pos', 'slider_pose_near', 'slider_pose_above'])

		with _sm_move_to_slider_and_drop_1:
			# x:129 y:39
			OperatableStateMachine.add('Read above slider pose',
										ReadFromMongo(),
										transitions={'continue': 'Read near slider pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name_array': 'slider_pose_above', 'joints_data': 'tf_slider_pose_above'})

			# x:344 y:235
			OperatableStateMachine.add('Move near slider',
										CallJointTrap(max_vel=0.5, max_acl=0.5, namespace="panda_1"),
										transitions={'continue': 'Release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_slider_pose_near', 'joint_values': 'minjerk_out'})

			# x:339 y:27
			OperatableStateMachine.add('Read near slider pose',
										ReadFromMongo(),
										transitions={'continue': 'Move above slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name_array': 'slider_pose_near', 'joints_data': 'tf_slider_pose_near'})

			# x:342 y:338
			OperatableStateMachine.add('Release object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'release_pos', 'success': 'success'})

			# x:342 y:147
			OperatableStateMachine.add('Move above slider',
										CallJointTrap(max_vel=0.5, max_acl=0.5, namespace="panda_1"),
										transitions={'continue': 'Move near slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_slider_pose_above', 'joint_values': 'minjerk_out'})


		# x:30 y:365, x:130 y:365
		_sm_move_to_object_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['init_pose', 'pickup_pose'])

		with _sm_move_to_object_2:
			# x:206 y:24
			OperatableStateMachine.add('Read init pose',
										ReadFromMongo(),
										transitions={'continue': 'Read pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name_array': 'init_pose', 'joints_data': 'tf_init_pose'})

			# x:453 y:196
			OperatableStateMachine.add('Move pickup pose',
										CallJointTrap(max_vel=0.5, max_acl=0.5, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_pickup_pose', 'joint_values': 'minjerk_out'})

			# x:455 y:27
			OperatableStateMachine.add('Read pickup pose',
										ReadFromMongo(),
										transitions={'continue': 'Move init pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name_array': 'pickup_pose', 'joints_data': 'tf_pickup_pose'})

			# x:456 y:116
			OperatableStateMachine.add('Move init pose',
										CallJointTrap(max_vel=0.5, max_acl=0.5, namespace="panda_1"),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_init_pose', 'joint_values': 'minjerk_out'})


		# x:30 y:365, x:130 y:365
		_sm_init_cell_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos'])

		with _sm_init_cell_3:
			# x:126 y:133
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'Open hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:365 y:128
			OperatableStateMachine.add('Open hand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_pos', 'success': 'success'})


		# x:794 y:301, x:223 y:588
		_sm_close_and_rotate_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false'])

		with _sm_close_and_rotate_4:
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
			# x:149 y:29
			OperatableStateMachine.add('Init cell',
										_sm_init_cell_3,
										transitions={'finished': 'Move to object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos'})

			# x:417 y:123
			OperatableStateMachine.add('Grab object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'Move to slider and drop', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'grab_pos', 'success': 'success'})

			# x:424 y:36
			OperatableStateMachine.add('Move to object',
										_sm_move_to_object_2,
										transitions={'finished': 'Grab object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'init_pose': 'init_pose', 'pickup_pose': 'pickup_pose'})

			# x:411 y:229
			OperatableStateMachine.add('Move to slider and drop',
										_sm_move_to_slider_and_drop_1,
										transitions={'finished': 'Pickup from holder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'release_pos': 'release_pos', 'slider_pose_near': 'slider_pose_near', 'slider_pose_above': 'slider_pose_above'})

			# x:421 y:333
			OperatableStateMachine.add('Pickup from holder',
										_sm_pickup_from_holder_0,
										transitions={'finished': 'failed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grab_pos': 'grab_pos', 'holder_pickup_pose': 'holder_pickup_pose', 'holder_up_pose': 'holder_up_pose'})

			# x:412 y:427
			OperatableStateMachine.add('Close and rotate',
										_sm_close_and_rotate_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
