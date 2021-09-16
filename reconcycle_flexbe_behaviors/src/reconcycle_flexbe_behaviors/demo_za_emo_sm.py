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
from reconcycle_flexbe_states.Drop_Random_On_Table import DropRandomOnTable
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand
from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin
from reconcycle_flexbe_states.active_controller_service_client import ActiveControllerProxyClient
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
from reconcycle_flexbe_states.set_cartesian_compliance_reconcycle import SetReconcycleCartesianCompliance
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
		self.add_parameter('Kp', 2000)
		self.add_parameter('Kr', 30)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:467 y:697, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grab_pos = [0.8]
		_state_machine.userdata.true = True
		_state_machine.userdata.false = False
		_state_machine.userdata.release_pos = [0.2]
		_state_machine.userdata.holder_up_pose = "emo/j_holder_up_pose"
		_state_machine.userdata.j_pickup_pose = "emo/j_holder_pickup_pose"
		_state_machine.userdata.namespace = "panda_1"
		_state_machine.userdata.j_init_pose = "emo/j_init_pose"
		_state_machine.userdata.j_above_table = "emo/j_above_table"
		_state_machine.userdata.j_between_slider = "emo/j_between_slider"
		_state_machine.userdata.j_above_slider = "emo/j_above_slider"
		_state_machine.userdata.j_slightly_above_slider = "emo/j_slightly_above_slider"
		_state_machine.userdata.rotation = [0,0,0]
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.holder_pickup_pose = "emo/holder_pickup_pose"

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
			# x:536 y:26
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:792 y:25
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:841 y:632, x:86 y:481
		_sm_joint_place_to_slider_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_joint_place_to_slider_2:
			# x:786 y:39
			OperatableStateMachine.add('read_j_above_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_above_slider'})

			# x:780 y:142
			OperatableStateMachine.add('move_j_above_slider',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'read_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})

			# x:230 y:77
			OperatableStateMachine.add('move_j_above_table',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'read_j_between_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:599 y:70
			OperatableStateMachine.add('move_j_between_slider',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'read_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:754 y:314
			OperatableStateMachine.add('move_j_slightly_above_slider',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:38 y:84
			OperatableStateMachine.add('read_j_above_table',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'joints_data'})

			# x:415 y:71
			OperatableStateMachine.add('read_j_between_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_between_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_between_slider', 'joints_data': 'joints_data'})

			# x:753 y:226
			OperatableStateMachine.add('read_j_slightly_above_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_slightly_above_slider', 'joints_data': 'joints_data'})

			# x:773 y:407
			OperatableStateMachine.add('release object',
										MoveSoftHand(motion_duration=1, motion_timestep=0.02),
										transitions={'continue': 'move_back_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'release_pos', 'success': 'success'})

			# x:773 y:484
			OperatableStateMachine.add('move_back_above_slider',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})


		# x:894 y:652, x:130 y:365
		_sm_joint_move_above_table_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'offset', 'rotation'], output_keys=['tf_pickup_pose', 'mdb_above_table_pose'])

		with _sm_joint_move_above_table_3:
			# x:556 y:54
			OperatableStateMachine.add('Read object TF',
										ReadTFCartLin(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_link0"),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})

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
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})


		# x:214 y:382, x:206 y:267
		_sm_cart_move_to_object_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos'])

		with _sm_cart_move_to_object_4:
			# x:408 y:50
			OperatableStateMachine.add('Move pickup above pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[-0.05,0.04,-0.2,-102.472,-3.952,143], offset_type='local', limit_rotations=False),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:414 y:402
			OperatableStateMachine.add('Move back pickup',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[-0.05,0.04,-0.2,-102.472,-3.952,143], offset_type='local', limit_rotations=False),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:412 y:157
			OperatableStateMachine.add('Move pickup pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=3, offset=[-0.05,0.04,-0.06,-102.472,-3.952,143], offset_type='local', limit_rotations=False),
										transitions={'continue': 'Grab the object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:418 y:284
			OperatableStateMachine.add('Grab the object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.05),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'grab_pos', 'success': 'success'})


		# x:764 y:346, x:457 y:187
		_sm_place_randomly_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['mdb_above_table_pose', 'mdb_init_pose'])

		with _sm_place_randomly_5:
			# x:95 y:41
			OperatableStateMachine.add('Move init',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'Move above table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:309 y:42
			OperatableStateMachine.add('Move above table',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'switch cartesian', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:952 y:203
			OperatableStateMachine.add('Move back above',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:737 y:12
			OperatableStateMachine.add('set compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'Drop randomly', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:532 y:25
			OperatableStateMachine.add('switch cartesian',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:995 y:65
			OperatableStateMachine.add('Drop randomly',
										DropRandomOnTable(namespace="panda_1", exe_time=3),
										transitions={'continue': 'Move back above', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_out': 't2_out'})


		# x:912 y:484, x:1068 y:31
		_sm_pickup_from_holder_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['grab_pos', 'holder_pickup_pose', 'holder_up_pose', 'false'])

		with _sm_pickup_from_holder_6:
			# x:43 y:58
			OperatableStateMachine.add('move_slider_back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'read_holder_above', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:371 y:88
			OperatableStateMachine.add('move_above_holder',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'read_holder_pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_holder_above', 'joint_values': 'joint_values'})

			# x:780 y:304
			OperatableStateMachine.add('move_above_holder_again',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_holder_above', 'joint_values': 'joint_values'})

			# x:779 y:81
			OperatableStateMachine.add('move_to_holder',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'grasp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'tf_holder_pickup', 'joint_values': 'joint_values'})

			# x:200 y:86
			OperatableStateMachine.add('read_holder_above',
										ReadFromMongo(),
										transitions={'continue': 'move_above_holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'holder_up_pose', 'joints_data': 'tf_holder_above'})

			# x:569 y:80
			OperatableStateMachine.add('read_holder_pickup',
										ReadFromMongo(),
										transitions={'continue': 'move_to_holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'holder_pickup_pose', 'joints_data': 'tf_holder_pickup'})

			# x:784 y:185
			OperatableStateMachine.add('grasp',
										MoveSoftHand(motion_duration=1, motion_timestep=0.1),
										transitions={'continue': 'move_above_holder_again', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'grab_pos', 'success': 'success'})


		# x:994 y:572, x:562 y:500
		_sm_init_cell_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos', 'value', 'false', 'j_init_pose'], output_keys=['mdb_init_pose'])

		with _sm_init_cell_7:
			# x:46 y:51
			OperatableStateMachine.add('Load_cartesian_contr',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Load_joint_contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:222 y:50
			OperatableStateMachine.add('Load_joint_contr',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'error_recovery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:976 y:107
			OperatableStateMachine.add('Open hand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_pos', 'success': 'success'})

			# x:611 y:133
			OperatableStateMachine.add('Open holder',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'move_slider_to_front', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:20 y:225
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'move_slider_back', 'failed': 'switch to joint'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:915 y:12
			OperatableStateMachine.add('Read init',
										ReadFromMongo(),
										transitions={'continue': 'move init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'mdb_init_pose'})

			# x:456 y:47
			OperatableStateMachine.add('error_recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'switch to joint', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1109 y:11
			OperatableStateMachine.add('move init',
										CallJointTrap(max_vel=1, max_acl=1, namespace='panda_1'),
										transitions={'continue': 'Open pneumatic block', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:204 y:224
			OperatableStateMachine.add('move_slider_back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:773 y:130
			OperatableStateMachine.add('move_slider_to_front',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Open hand', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:427 y:133
			OperatableStateMachine.add('rotate up',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'wait_1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:695 y:47
			OperatableStateMachine.add('switch to joint',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller"], strictness=1),
										transitions={'continue': 'Read init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:475 y:231
			OperatableStateMachine.add('wait_1',
										WaitState(wait_time=1),
										transitions={'done': 'Open holder'},
										autonomy={'done': Autonomy.Off})

			# x:353 y:228
			OperatableStateMachine.add('wait_2',
										WaitState(wait_time=2),
										transitions={'done': 'rotate up'},
										autonomy={'done': Autonomy.Off})


		# x:833 y:614, x:223 y:588
		_sm_close_and_rotate_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'value'])

		with _sm_close_and_rotate_8:
			# x:60 y:31
			OperatableStateMachine.add('move_slider_back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:787 y:320
			OperatableStateMachine.add('open',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:636 y:25
			OperatableStateMachine.add('rotate down',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'wait1_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:794 y:143
			OperatableStateMachine.add('rotate up',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'wait1_3', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:220 y:18
			OperatableStateMachine.add('wait0',
										WaitState(wait_time=2),
										transitions={'done': 'close'},
										autonomy={'done': Autonomy.Off})

			# x:503 y:21
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'rotate down'},
										autonomy={'done': Autonomy.Off})

			# x:842 y:19
			OperatableStateMachine.add('wait1_2',
										WaitState(wait_time=3),
										transitions={'done': 'rotate up'},
										autonomy={'done': Autonomy.Off})

			# x:844 y:236
			OperatableStateMachine.add('wait1_3',
										WaitState(wait_time=1),
										transitions={'done': 'open'},
										autonomy={'done': Autonomy.Off})

			# x:324 y:21
			OperatableStateMachine.add('close',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})



		with _state_machine:
			# x:94 y:26
			OperatableStateMachine.add('Init cell',
										_sm_init_cell_7,
										transitions={'finished': 'joint_Move_above_table', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos', 'value': 'true', 'false': 'false', 'j_init_pose': 'j_init_pose', 'mdb_init_pose': 'mdb_init_pose'})

			# x:1046 y:345
			OperatableStateMachine.add('Pickup from holder',
										_sm_pickup_from_holder_6,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'grab_pos': 'grab_pos', 'holder_pickup_pose': 'j_pickup_pose', 'holder_up_pose': 'holder_up_pose', 'false': 'false'})

			# x:843 y:560
			OperatableStateMachine.add('Place randomly',
										_sm_place_randomly_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'mdb_above_table_pose': 'mdb_above_table_pose', 'mdb_init_pose': 'mdb_init_pose'})

			# x:1017 y:10
			OperatableStateMachine.add('cart_Move to object',
										_sm_cart_move_to_object_4,
										transitions={'finished': 'start_joint_imp_controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos'})

			# x:481 y:11
			OperatableStateMachine.add('joint_Move_above_table',
										_sm_joint_move_above_table_3,
										transitions={'finished': 'start_cartesian_imp_controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'mdb_above_table_pose': 'mdb_above_table_pose'})

			# x:1041 y:229
			OperatableStateMachine.add('joint_place_to_slider',
										_sm_joint_place_to_slider_2,
										transitions={'finished': 'Pickup from holder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:736 y:11
			OperatableStateMachine.add('start_cartesian_imp_controller',
										_sm_start_cartesian_imp_controller_1,
										transitions={'finished': 'cart_Move to object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1010 y:129
			OperatableStateMachine.add('start_joint_imp_controller',
										_sm_start_joint_imp_controller_0,
										transitions={'finished': 'joint_place_to_slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1036 y:484
			OperatableStateMachine.add('Close and rotate',
										_sm_close_and_rotate_8,
										transitions={'finished': 'Pickup from holder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'value': 'false'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
