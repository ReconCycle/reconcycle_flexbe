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
from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin
from reconcycle_flexbe_states.active_controller_service_client import ActiveControllerProxyClient
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
from reconcycle_flexbe_states.read_from_mongodb_POSE import ReadFromMongoPOSE
from reconcycle_flexbe_states.set_cartesian_impedance_service_client import SetCartesianImpedanceProxyClient
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
		_state_machine.userdata.slider_pose_above = "emo/j_slider_pose_above"
		_state_machine.userdata.slider_pose_near = "emo/j_slider_pose_near"
		_state_machine.userdata.holder_pickup_pose = "emo/j_holder_pickup_pose"
		_state_machine.userdata.holder_up_pose = "emo/j_holder_up_pose"
		_state_machine.userdata.j_pickup_pose = "emo/j_above_pickup_pose"
		_state_machine.userdata.namespace = "panda_1"
		_state_machine.userdata.j_init_pose = "emo/j_init_pose"
		_state_machine.userdata.j_above_table = "emo/j_above_table"
		_state_machine.userdata.j_between_slider = "emo/j_between_slider"
		_state_machine.userdata.j_above_slider = "emo/j_above_slider"
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.rotation = [0,0,0]

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


		# x:1101 y:44, x:365 y:293
		_sm_start_cartesian_imp_controller_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_imp_controller_1:
			# x:548 y:49
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set cartesian impedance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:83 y:354
			OperatableStateMachine.add('load_cartesian_controller',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:782 y:44
			OperatableStateMachine.add('set cartesian impedance',
										SetCartesianImpedanceProxyClient(cartesian_stiffness=[2000,2000,2000,100,100,100], robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:76 y:194
			OperatableStateMachine.add('find_current_controller',
										ActiveControllerProxyClient(robot_name="/panda_1", real_controllers="joint_impedance_controller"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:857 y:435, x:86 y:481
		_sm_joint_place_to_slider_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider'])

		with _sm_joint_place_to_slider_2:
			# x:46 y:32
			OperatableStateMachine.add('read_j_above_table',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'joints_data'})

			# x:233 y:36
			OperatableStateMachine.add('move_j_above_table',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'read_j_between_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:599 y:27
			OperatableStateMachine.add('move_j_between_slider',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'read_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:786 y:39
			OperatableStateMachine.add('read_j_above_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_data'})

			# x:413 y:28
			OperatableStateMachine.add('read_j_between_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_between_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_between_slider', 'joints_data': 'joints_data'})

			# x:753 y:200
			OperatableStateMachine.add('move_j_above_slider',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})


		# x:894 y:652, x:130 y:365
		_sm_joint_move_above_table_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_init_pose', 'j_above_table', 'offset', 'rotation'], output_keys=['tf_pickup_pose'])

		with _sm_joint_move_above_table_3:
			# x:141 y:45
			OperatableStateMachine.add('Read_init',
										ReadFromMongo(),
										transitions={'continue': 'j_move_to_init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'mdb_init_pose'})

			# x:762 y:81
			OperatableStateMachine.add('Read_above_table',
										ReadFromMongo(),
										transitions={'continue': 'j_move_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'mdb_above_table_pose'})

			# x:878 y:242
			OperatableStateMachine.add('j_move_above_table',
										CallJointTrap(max_vel=1, max_acl=1, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:324 y:45
			OperatableStateMachine.add('j_move_to_init',
										CallJointTrap(max_vel=1, max_acl=1, namespace="panda_1"),
										transitions={'continue': 'Read object TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:504 y:52
			OperatableStateMachine.add('Read object TF',
										ReadTFCartLin(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_link0"),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})


		# x:214 y:382, x:206 y:267
		_sm_cart_move_to_object_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose'])

		with _sm_cart_move_to_object_4:
			# x:408 y:50
			OperatableStateMachine.add('Move pickup above pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[0,0,0.2,0,0,1,0]),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 'minjerk_out'})

			# x:413 y:183
			OperatableStateMachine.add('Move pickup pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[0,0,0.0,0,0,1,0]),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:425 y:310
			OperatableStateMachine.add('Move back pickup',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[0,0,0.2,0,0,1,0]),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})


		# x:30 y:365, x:164 y:172
		_sm_pickup_from_holder_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grab_pos', 'holder_pickup_pose', 'holder_up_pose'])

		with _sm_pickup_from_holder_5:
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
		_sm_move_to_slider_and_drop_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['release_pos', 'slider_pose_near', 'slider_pose_above'])

		with _sm_move_to_slider_and_drop_6:
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


		# x:1195 y:247, x:616 y:355
		_sm_init_cell_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos'])

		with _sm_init_cell_7:
			# x:105 y:54
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'Open hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:738 y:48
			OperatableStateMachine.add('Load_joint_contr',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch to joint', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:319 y:52
			OperatableStateMachine.add('Open hand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'Load_cartesian_contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_pos', 'success': 'success'})

			# x:962 y:161
			OperatableStateMachine.add('error_recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'switch to joint'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:962 y:41
			OperatableStateMachine.add('switch to joint',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller"], strictness=1),
										transitions={'continue': 'error_recovery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:532 y:31
			OperatableStateMachine.add('Load_cartesian_contr',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Load_joint_contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:794 y:301, x:223 y:588
		_sm_close_and_rotate_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false'])

		with _sm_close_and_rotate_8:
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
			# x:97 y:25
			OperatableStateMachine.add('Init cell',
										_sm_init_cell_7,
										transitions={'finished': 'joint_Move_above_table', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos'})

			# x:1042 y:25
			OperatableStateMachine.add('Grab object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'start_joint_imp_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'grab_pos', 'success': 'success'})

			# x:1033 y:337
			OperatableStateMachine.add('Move to slider and drop',
										_sm_move_to_slider_and_drop_6,
										transitions={'finished': 'Pickup from holder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'release_pos': 'release_pos', 'slider_pose_near': 'slider_pose_near', 'slider_pose_above': 'slider_pose_above'})

			# x:1006 y:431
			OperatableStateMachine.add('Pickup from holder',
										_sm_pickup_from_holder_5,
										transitions={'finished': 'Close and rotate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grab_pos': 'grab_pos', 'holder_pickup_pose': 'holder_pickup_pose', 'holder_up_pose': 'holder_up_pose'})

			# x:782 y:14
			OperatableStateMachine.add('cart_Move to object',
										_sm_cart_move_to_object_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose'})

			# x:267 y:16
			OperatableStateMachine.add('joint_Move_above_table',
										_sm_joint_move_above_table_3,
										transitions={'finished': 'start_cartesian_imp_controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose', 'j_above_table': 'j_above_table', 'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose'})

			# x:1041 y:229
			OperatableStateMachine.add('joint_place_to_slider',
										_sm_joint_place_to_slider_2,
										transitions={'finished': 'Move to slider and drop', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider'})

			# x:500 y:14
			OperatableStateMachine.add('start_cartesian_imp_controller',
										_sm_start_cartesian_imp_controller_1,
										transitions={'finished': 'cart_Move to object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1010 y:129
			OperatableStateMachine.add('start_joint_imp_controller',
										_sm_start_joint_imp_controller_0,
										transitions={'finished': 'joint_place_to_slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:992 y:547
			OperatableStateMachine.add('Close and rotate',
										_sm_close_and_rotate_8,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
