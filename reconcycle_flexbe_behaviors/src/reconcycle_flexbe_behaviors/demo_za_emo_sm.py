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
from reconcycle_flexbe_states.CallAction_JointTrapVel import CallJointTrap
from reconcycle_flexbe_states.CallAction_TF_CartLin import CallActionTFCartLin
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand
from reconcycle_flexbe_states.active_controller_service_client import ActiveControllerProxyClient
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
from reconcycle_flexbe_states.read_from_mongodb_POSE import ReadFromMongoPOSE
from reconcycle_flexbe_states.switch_controller_service_client import SwitchControllerProxyClient
from reconcycle_flexbe_states.unload_controller_service_client import UnloadControllerProxyClient
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
		# x:467 y:697, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grab_pos = [0.6]
		_state_machine.userdata.true = True
		_state_machine.userdata.false = False
		_state_machine.userdata.release_pos = [0.2]
		_state_machine.userdata.init_pose = "emo/init_pose"
		_state_machine.userdata.pickup_pose = "emo/pickup_pose"
		_state_machine.userdata.trans_pickup_pose = "emo/trans_pickup_pose"
		_state_machine.userdata.slider_pose_above = "emo/j_slider_pose_above"
		_state_machine.userdata.slider_pose_near = "emo/j_slider_pose_near"
		_state_machine.userdata.holder_pickup_pose = "emo/j_holder_pickup_pose"
		_state_machine.userdata.holder_up_pose = "emo/j_holder_up_pose"
		_state_machine.userdata.j_pickup_pose = "emo/j_above_pickup_pose"
		_state_machine.userdata.j_init_pose = "emo/j_init_pose"
		_state_machine.userdata.namespace = "panda_1"
		_state_machine.userdata.j_above_table = "emo/j_above_table"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:889 y:225, x:467 y:299
		_sm_start_joint_imp_controller_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_imp_controller_0:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller"], strictness=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:98 y:430
			OperatableStateMachine.add('load_joint_controller',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:614 y:368
			OperatableStateMachine.add('unload_cartesian_controller',
										UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:115 y:326
			OperatableStateMachine.add('find_active_controller',
										ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:862 y:192, x:365 y:293
		_sm_start_cartesian_imp_controller_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_imp_controller_1:
			# x:548 y:49
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:83 y:354
			OperatableStateMachine.add('load_cartesian_controller',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:76 y:194
			OperatableStateMachine.add('find_current_controller',
										ActiveControllerProxyClient(robot_name="/panda_1", real_controllers="joint_impedance_controller"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:894 y:652, x:130 y:365
		_sm_joint_move_above_table_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_init_pose', 'j_above_table'])

		with _sm_joint_move_above_table_2:
			# x:141 y:45
			OperatableStateMachine.add('Read_init',
										ReadFromMongo(),
										transitions={'continue': 'j_move_to_init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'mdb_init_pose'})

			# x:760 y:34
			OperatableStateMachine.add('j_move_above_table',
										CallJointTrap(max_vel=1, max_acl=1, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:327 y:39
			OperatableStateMachine.add('j_move_to_init',
										CallJointTrap(max_vel=1, max_acl=1, namespace="panda_1"),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:562 y:35
			OperatableStateMachine.add('Read_above_table',
										ReadFromMongo(),
										transitions={'continue': 'j_move_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'mdb_above_table_pose'})


		# x:214 y:382, x:346 y:144
		_sm_cart_move_to_object_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['init_pose', 'trans_pickup_pose', 'pickup_pose', 'j_pickup_pose'])

		with _sm_cart_move_to_object_3:
			# x:77 y:30
			OperatableStateMachine.add('Read init pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'Move init pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'init_pose', 'joints_data': 'tf_init_pose'})

			# x:302 y:27
			OperatableStateMachine.add('Move init pose',
										CallActionTFCartLin(namespace="/panda_1", exe_time=3, offset=0),
										transitions={'continue': 'Read pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_init_pose', 't2_out': 'minjerk_out'})

			# x:717 y:143
			OperatableStateMachine.add('Move pickup above pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[0,0,0.2,0,0,0,1]),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 'minjerk_out'})

			# x:519 y:359
			OperatableStateMachine.add('Move pickup pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[0,0,0.15,0,0,0,1]),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:671 y:32
			OperatableStateMachine.add('Read pickup pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'Move pickup above pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'pickup_pose', 'joints_data': 'tf_pickup_pose'})

			# x:515 y:461
			OperatableStateMachine.add('Move back pickup',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[0,0,0.2,0,0,0,1]),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})


		# x:30 y:365, x:164 y:172
		_sm_pickup_from_holder_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grab_pos', 'holder_pickup_pose', 'holder_up_pose'])

		with _sm_pickup_from_holder_4:
			# x:99 y:42
			OperatableStateMachine.add('Read holder pickup',
										ReadFromMongoPOSE(),
										transitions={'continue': 'Read holder up', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'holder_pickup_pose', 'joints_data': 'tf_holder_pickup'})

			# x:297 y:273
			OperatableStateMachine.add('Move holder up',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=None),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_holder_up', 't2_out': 't2_out'})

			# x:302 y:40
			OperatableStateMachine.add('Read holder up',
										ReadFromMongoPOSE(),
										transitions={'continue': 'Move holder pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'holder_up_pose', 'joints_data': 'tf_holder_up'})

			# x:298 y:158
			OperatableStateMachine.add('Move holder pickup',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=None),
										transitions={'continue': 'Move holder up', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_holder_pickup', 't2_out': 't2_out'})


		# x:30 y:365, x:181 y:175
		_sm_move_to_slider_and_drop_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['release_pos', 'slider_pose_near', 'slider_pose_above'])

		with _sm_move_to_slider_and_drop_5:
			# x:129 y:39
			OperatableStateMachine.add('Read above slider pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'Read near slider pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'slider_pose_above', 'joints_data': 'tf_slider_pose_above'})

			# x:344 y:235
			OperatableStateMachine.add('Move near slider',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=None),
										transitions={'continue': 'Release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_slider_pose_near', 't2_out': 'minjerk_out'})

			# x:339 y:27
			OperatableStateMachine.add('Read near slider pose',
										ReadFromMongoPOSE(),
										transitions={'continue': 'Move above slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'slider_pose_near', 'joints_data': 'tf_slider_pose_near'})

			# x:342 y:338
			OperatableStateMachine.add('Release object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'release_pos', 'success': 'success'})

			# x:342 y:147
			OperatableStateMachine.add('Move above slider',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=None),
										transitions={'continue': 'Move near slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_slider_pose_above', 't2_out': 'minjerk_out'})


		# x:1174 y:62, x:616 y:355
		_sm_init_cell_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos'])

		with _sm_init_cell_6:
			# x:105 y:54
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'Open hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:738 y:48
			OperatableStateMachine.add('Load_joint_contr',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'error_recovery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:319 y:52
			OperatableStateMachine.add('Open hand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'Load_cartesian_contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_pos', 'success': 'success'})

			# x:945 y:50
			OperatableStateMachine.add('error_recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:532 y:31
			OperatableStateMachine.add('Load_cartesian_contr',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Load_joint_contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:794 y:301, x:223 y:588
		_sm_close_and_rotate_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false'])

		with _sm_close_and_rotate_7:
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
			# x:257 y:21
			OperatableStateMachine.add('joint_Move_above_table',
										_sm_joint_move_above_table_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose', 'j_above_table': 'j_above_table'})

			# x:780 y:122
			OperatableStateMachine.add('Grab object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'start_joint_imp_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'grab_pos', 'success': 'success'})

			# x:97 y:25
			OperatableStateMachine.add('Init cell',
										_sm_init_cell_6,
										transitions={'finished': 'joint_Move_above_table', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos'})

			# x:441 y:260
			OperatableStateMachine.add('Move to slider and drop',
										_sm_move_to_slider_and_drop_5,
										transitions={'finished': 'Pickup from holder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'release_pos': 'release_pos', 'slider_pose_near': 'slider_pose_near', 'slider_pose_above': 'slider_pose_above'})

			# x:436 y:362
			OperatableStateMachine.add('Pickup from holder',
										_sm_pickup_from_holder_4,
										transitions={'finished': 'failed', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grab_pos': 'grab_pos', 'holder_pickup_pose': 'holder_pickup_pose', 'holder_up_pose': 'holder_up_pose'})

			# x:784 y:23
			OperatableStateMachine.add('cart_Move to object',
										_sm_cart_move_to_object_3,
										transitions={'finished': 'Grab object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'init_pose': 'init_pose', 'trans_pickup_pose': 'trans_pickup_pose', 'pickup_pose': 'pickup_pose', 'j_pickup_pose': 'j_pickup_pose'})

			# x:471 y:22
			OperatableStateMachine.add('start_cartesian_imp_controller',
										_sm_start_cartesian_imp_controller_1,
										transitions={'finished': 'cart_Move to object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:768 y:218
			OperatableStateMachine.add('start_joint_imp_controller',
										_sm_start_joint_imp_controller_0,
										transitions={'finished': 'finished', 'failed': 'Pickup from holder'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:434 y:486
			OperatableStateMachine.add('Close and rotate',
										_sm_close_and_rotate_7,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
