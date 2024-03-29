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
from reconcycle_flexbe_states.PickUpHCA_TF_CartLin import PickUpHCACartLin
from reconcycle_flexbe_states.ReadNextVisionAction import ReadNextVisionAction
from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin
from reconcycle_flexbe_states.Read_TF_HCA import ReadTFHCA
from reconcycle_flexbe_states.SelectAction import ReadAction
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
Created on Tue May 03 2022
@author: Matija Mavsar
'''
class GenericdisassemblyprotocolSM(Behavior):
	'''
	Development version of a generic disassembly protocol for all HCAs
	'''


	def __init__(self):
		super(GenericdisassemblyprotocolSM, self).__init__()
		self.name = 'Generic disassembly protocol'

		# parameters of this behavior
		self.add_parameter('max_acl', 1)
		self.add_parameter('max_vel', 1)
		self.add_parameter('Kp', 2000)
		self.add_parameter('Kr', 30)
		self.add_parameter('cartesian_controller', 'cartesian_impedance_controller')
		self.add_parameter('namespace', 'panda_1')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		soft_hand_quat_offset = [0.888, 0.458, 0.020, -0.044]
		soft_hand_pos_offset = [0.01, -0.05, 0.0]
		pin_x = 0.004
		pin_y = -0.001
		# x:918 y:239, x:382 y:312
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.true = True
		_state_machine.userdata.false = False
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.rotation = [0,0,0]
		_state_machine.userdata.release_pos = [0.1]
		_state_machine.userdata.safe_position_name = "tmp"
		_state_machine.userdata.grab_pos = [0.8]
		_state_machine.userdata.soft_grab_pos = [0.9]
		_state_machine.userdata.j_push_pose = "panda_2/joints/pin_push"
		_state_machine.userdata.j_above_vision = "panda_1/joints/above_vision"
		_state_machine.userdata.j_between_slider = "tmp"
		_state_machine.userdata.j_above_slider = "tmp"
		_state_machine.userdata.j_slightly_above_slider = "panda_1/joints/slightly_above_slider"
		_state_machine.userdata.j_above_holder = "panda_1/joints/above_holder"
		_state_machine.userdata.j_above_table = "tmp"
		_state_machine.userdata.j_init_pose = "new/init_joints"
		_state_machine.userdata.j_init2_pose = "panda_2/joints/init"
		_state_machine.userdata.j_lever_pose = "panda_2/joints/above_levering"
		_state_machine.userdata.hca_location = "vision_table"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_rotate_down_and_up_0 = OperatableStateMachine(outcomes=['failed', 'done'], input_keys=['true', 'false'])

		with _sm_rotate_down_and_up_0:
			# x:492 y:105
			OperatableStateMachine.add('Rotate holder down',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:156 y:95
			OperatableStateMachine.add('Rotate holder up',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'Wait_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:349 y:102
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=3),
										transitions={'done': 'Rotate holder up'},
										autonomy={'done': Autonomy.Off})

			# x:30 y:40
			OperatableStateMachine.add('Wait_2',
										WaitState(wait_time=3),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_1:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_2", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:81 y:195
			OperatableStateMachine.add('load_joint_controller',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_2"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:336
			OperatableStateMachine.add('unload_cartesian_controller',
										UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_2"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:73 y:108
			OperatableStateMachine.add('find_active_controller',
										ActiveControllerProxyClient(robot_name="panda_2", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:130 y:365
		_sm_return_panda_2_to_init_2 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init2_pose'])

		with _sm_return_panda_2_to_init_2:
			# x:30 y:40
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_1,
										transitions={'finished': 'Read init2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:325 y:145
			OperatableStateMachine.add('move push',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_2"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init2_joints', 'joint_values': 'joint_values'})

			# x:341 y:53
			OperatableStateMachine.add('Read init2',
										ReadFromMongo(),
										transitions={'continue': 'move push', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init2_pose', 'joints_data': 'mdb_init2_joints'})


		# x:414 y:254, x:553 y:619
		_sm_push_pin_out_3 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_push_pose', 'offset', 'rotation', 'true', 'j_init2_pose', 'false'])

		with _sm_push_pin_out_3:
			# x:49 y:40
			OperatableStateMachine.add('Read push',
										ReadFromMongo(),
										transitions={'continue': 'move push', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_push_pose', 'joints_data': 'mdb_push_pose'})

			# x:1025 y:430
			OperatableStateMachine.add('Move push BACK',
										CallActionTFCartLin(namespace='panda_2', exe_time=3, local_offset=[pin_x,pin_y,-0.07,0,0,0], global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Read back', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})

			# x:723 y:596
			OperatableStateMachine.add('Move push BACK_2',
										CallActionTFCartLin(namespace='panda_2', exe_time=3, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push_back', 't2_out': 't2_out'})

			# x:1020 y:227
			OperatableStateMachine.add('Move push TF',
										CallActionTFCartLin(namespace='panda_2', exe_time=3, local_offset=[pin_x,pin_y,0.002,0,0,0], global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Move push ALL THE WAY', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})

			# x:899 y:516
			OperatableStateMachine.add('Read back',
										ReadTFCartLin(target_frame='panda_2/pose/pin_push_back', source_frame='panda_2/panda_2_link0'),
										transitions={'continue': 'Move push BACK_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_push_back'})

			# x:1046 y:133
			OperatableStateMachine.add('Read push TF',
										ReadTFCartLin(target_frame='panda_2/pose/pin_push', source_frame='panda_2/panda_2_link0'),
										transitions={'continue': 'Move push TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_push'})

			# x:731 y:446
			OperatableStateMachine.add('Return panda_2 to init',
										_sm_return_panda_2_to_init_2,
										transitions={'failed': 'failed', 'continue': 'open'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init2_pose': 'j_init2_pose'})

			# x:291 y:462
			OperatableStateMachine.add('Rotate down and up',
										_sm_rotate_down_and_up_0,
										transitions={'failed': 'failed', 'done': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'done': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})

			# x:394 y:32
			OperatableStateMachine.add('close',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:217 y:32
			OperatableStateMachine.add('move push',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_2"),
										transitions={'continue': 'close', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_push_pose', 'joint_values': 'joint_values'})

			# x:509 y:417
			OperatableStateMachine.add('open',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:771 y:32
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_2", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:573 y:31
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name='panda_2', start_controller=[self.cartesian_controller], stop_controller=["joint_impedance_controller"], strictness=2),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1037 y:27
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'Read push TF'},
										autonomy={'done': Autonomy.Off})

			# x:1017 y:320
			OperatableStateMachine.add('Move push ALL THE WAY',
										CallActionTFCartLin(namespace='panda_2', exe_time=0.2, local_offset=[pin_x,pin_y,0.017,0,0,0], global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Move push BACK', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})


		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_imp_controller_4 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_imp_controller_4:
			# x:217 y:35
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=[self.cartesian_controller], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:139
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:460 y:35
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_5 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_5:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:81 y:195
			OperatableStateMachine.add('load_joint_controller',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:336
			OperatableStateMachine.add('unload_cartesian_controller',
										UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:139
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:73 y:108
			OperatableStateMachine.add('find_active_controller',
										ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_imp_controller_6 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_imp_controller_6:
			# x:217 y:35
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=[self.cartesian_controller], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:139
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:460 y:35
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[1500, 1500, 1500], Kr=[20, 20, 20]),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_7 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_7:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:81 y:195
			OperatableStateMachine.add('load_joint_controller',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:336
			OperatableStateMachine.add('unload_cartesian_controller',
										UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:73 y:108
			OperatableStateMachine.add('find_active_controller',
										ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:358 y:357
		_sm_place_hca_on_slider_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos', 'offset', 'rotation', 'j_above_holder'])

		with _sm_place_hca_on_slider_8:
			# x:747 y:104
			OperatableStateMachine.add('read_j_above_holder',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_holder', 'joints_data': 'joints_above_holder'})

			# x:858 y:355
			OperatableStateMachine.add('Read above holder TF',
										ReadTFCartLin(target_frame='panda_1/pose/above_holder', source_frame='panda_1/panda_1_link0'),
										transitions={'continue': 'Move above holder TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_above_holder'})

			# x:391 y:519
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_7,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:743 y:178
			OperatableStateMachine.add('move_j_above_holder',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'start_cartesian_imp_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_holder', 'joint_values': 'joint_values'})

			# x:585 y:33
			OperatableStateMachine.add('move_j_above_slider',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'read_j_above_holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})

			# x:265 y:627
			OperatableStateMachine.add('move_j_back_above_holder',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_holder', 'joint_values': 'joint_values'})

			# x:425 y:28
			OperatableStateMachine.add('read_j_above_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_above_slider'})

			# x:678 y:523
			OperatableStateMachine.add('release object',
										MoveSoftHand(motion_duration=1, motion_timestep=0.02),
										transitions={'continue': 'Start joint impedance controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'release_pos', 'success': 'success'})

			# x:726 y:260
			OperatableStateMachine.add('start_cartesian_imp_controller',
										_sm_start_cartesian_imp_controller_6,
										transitions={'finished': 'Read above holder TF', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:885 y:456
			OperatableStateMachine.add('Move above holder TF',
										CallActionTFCartLin(namespace='panda_1', exe_time=2, local_offset=[0,0.01,0,0,0,0], global_pos_offset=[0,0,0.01], limit_rotations=False),
										transitions={'continue': 'release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_above_holder', 't2_out': 't2_out'})


		# x:30 y:365, x:237 y:81
		_sm_move_above_vision_table_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_vision'])

		with _sm_move_above_vision_table_9:
			# x:44 y:40
			OperatableStateMachine.add('read_j_above_vision',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_vision', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_vision', 'joints_data': 'joints_above_vision'})

			# x:48 y:166
			OperatableStateMachine.add('move_j_above_vision',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_vision', 'joint_values': 'joint_values'})


		# x:504 y:546, x:366 y:236
		_sm_move_above_hca_and_pickup_10 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos'])

		with _sm_move_above_hca_and_pickup_10:
			# x:615 y:48
			OperatableStateMachine.add('Move above HCA',
										PickUpHCACartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=[0,0,0.2], limit_rotations=False, soft_hand_offset=[soft_hand_pos_offset, soft_hand_quat_offset]),
										transitions={'continue': 'Move to HCA', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:612 y:390
			OperatableStateMachine.add('Move back up',
										PickUpHCACartLin(namespace="panda_1", exe_time=1.5, local_offset=0, global_pos_offset=[0,0,0.2], limit_rotations=False, soft_hand_offset=[soft_hand_pos_offset, soft_hand_quat_offset]),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:609 y:166
			OperatableStateMachine.add('Move to HCA',
										PickUpHCACartLin(namespace="panda_1", exe_time=1.5, local_offset=[0.01,-0.0,0,0,0,0], global_pos_offset=[0,0,0.02], limit_rotations=False, soft_hand_offset=[soft_hand_pos_offset, soft_hand_quat_offset]),
										transitions={'continue': 'Grab the HCA', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:613 y:274
			OperatableStateMachine.add('Grab the HCA',
										MoveSoftHand(motion_duration=2, motion_timestep=0.05),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'soft_grab_pos', 'success': 'success'})


		# x:439 y:270, x:184 y:171
		_sm_close_vise_11 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'value'])

		with _sm_close_vise_11:
			# x:47 y:42
			OperatableStateMachine.add('move_slider_back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:248 y:56
			OperatableStateMachine.add('wait0',
										WaitState(wait_time=1),
										transitions={'done': 'close'},
										autonomy={'done': Autonomy.Off})

			# x:390 y:77
			OperatableStateMachine.add('close',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})


		# x:351 y:239, x:130 y:365
		_sm_pick_up_hca_and_put_into_vise_12 = OperatableStateMachine(outcomes=['failed', 'finished'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos', 'j_above_slider', 'j_slightly_above_slider', 'release_pos', 'true', 'false', 'j_between_slider', 'j_above_vision', 'j_above_holder', 'j_above_table', 'j_init_pose'])

		with _sm_pick_up_hca_and_put_into_vise_12:
			# x:58 y:166
			OperatableStateMachine.add('Move above vision table',
										_sm_move_above_vision_table_9,
										transitions={'finished': 'start_cartesian_imp_controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_vision': 'j_above_vision'})

			# x:444 y:63
			OperatableStateMachine.add('Move above HCA and pickup',
										_sm_move_above_hca_and_pickup_10,
										transitions={'finished': 'Start joint impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos'})

			# x:550 y:286
			OperatableStateMachine.add('Place HCA on slider',
										_sm_place_hca_on_slider_8,
										transitions={'finished': 'read_j_init', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos', 'offset': 'offset', 'rotation': 'rotation', 'j_above_holder': 'j_above_holder'})

			# x:504 y:169
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_5,
										transitions={'finished': 'Place HCA on slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:441 y:466
			OperatableStateMachine.add('move_j_init',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'Close vise', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_init', 'joint_values': 'joint_values'})

			# x:498 y:370
			OperatableStateMachine.add('read_j_init',
										ReadFromMongo(),
										transitions={'continue': 'move_j_init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'joints_init'})

			# x:169 y:73
			OperatableStateMachine.add('start_cartesian_imp_controller',
										_sm_start_cartesian_imp_controller_4,
										transitions={'finished': 'Move above HCA and pickup', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:384 y:575
			OperatableStateMachine.add('Close vise',
										_sm_close_vise_11,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'value': 'false'})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_2_13 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_2_13:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_2", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:81 y:195
			OperatableStateMachine.add('load_joint_controller',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_2"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:336
			OperatableStateMachine.add('unload_cartesian_controller',
										UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:139
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:73 y:108
			OperatableStateMachine.add('find_active_controller',
										ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_14 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_14:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_2", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:81 y:195
			OperatableStateMachine.add('load_joint_controller',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_2"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:336
			OperatableStateMachine.add('unload_cartesian_controller',
										UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:493 y:139
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:73 y:108
			OperatableStateMachine.add('find_active_controller',
										ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_impedance_controller_15 = OperatableStateMachine(outcomes=['continue', 'failed'])

		with _sm_start_cartesian_impedance_controller_15:
			# x:30 y:40
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name='panda_2', start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=2),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:134 y:111
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_2", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_location_2_16 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_init2_pose'])

		with _sm_move_to_safe_location_2_16:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init2_pose', 'joints_data': 'joints_safe_pos'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_2"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joints_data': 'joints_safe_pos', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_above_lever_17 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_lever_pose'])

		with _sm_move_above_lever_17:
			# x:30 y:47
			OperatableStateMachine.add('Read above lever',
										ReadFromMongo(),
										transitions={'continue': 'Move above lever', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_lever_pose', 'joints_data': 'mdb_lever_pose'})

			# x:215 y:40
			OperatableStateMachine.add('Move above lever',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_2"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_lever_pose', 'joint_values': 'joint_values'})


		# x:705 y:492, x:390 y:205
		_sm_levering_action_18 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'safe_position_name', 'j_lever_pose', 'j_init2_pose'])

		with _sm_levering_action_18:
			# x:30 y:166
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_14,
										transitions={'finished': 'Move above lever', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:1065 y:158
			OperatableStateMachine.add('Move to above gap pose',
										CallActionTFCartLin(namespace="panda_2", exe_time=3, local_offset=0, global_pos_offset=[0,0,0.05], limit_rotations=False),
										transitions={'continue': 'Move to gap pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_gap', 't2_out': 't2_out'})

			# x:671 y:406
			OperatableStateMachine.add('Move to above levered pose',
										CallActionTFCartLin(namespace="panda_2", exe_time=3, local_offset=0, global_pos_offset=[0.0,0.12,0.12], limit_rotations=False),
										transitions={'continue': 'Start joint impedance controller_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_levered_out', 't2_out': 't2_out'})

			# x:874 y:247
			OperatableStateMachine.add('Move to gap pose',
										CallActionTFCartLin(namespace="panda_2", exe_time=3, local_offset=0, global_pos_offset=[0.01,0,0], limit_rotations=False),
										transitions={'continue': 'Move to levered pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_gap', 't2_out': 't2_out'})

			# x:861 y:315
			OperatableStateMachine.add('Move to levered pose',
										CallActionTFCartLin(namespace="panda_2", exe_time=3, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Move to above levered pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_levered_out', 't2_out': 't2_out'})

			# x:282 y:553
			OperatableStateMachine.add('Move to safe location_2',
										_sm_move_to_safe_location_2_16,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_init2_pose': 'j_init2_pose'})

			# x:436 y:30
			OperatableStateMachine.add('Read gap pose',
										ReadTFCartLin(target_frame="vise/pose/hca_gap", source_frame="panda_2/panda_2_link0"),
										transitions={'continue': 'Start cartesian impedance controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_gap'})

			# x:996 y:53
			OperatableStateMachine.add('Read levered pose',
										ReadTFCartLin(target_frame="vise/pose/levered_out", source_frame="panda_2/panda_2_link0"),
										transitions={'continue': 'Move to gap pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_levered_out'})

			# x:693 y:26
			OperatableStateMachine.add('Start cartesian impedance controller',
										_sm_start_cartesian_impedance_controller_15,
										transitions={'continue': 'Read levered pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:417 y:426
			OperatableStateMachine.add('Start joint impedance controller_2',
										_sm_start_joint_impedance_controller_2_13,
										transitions={'finished': 'Move to safe location_2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:119 y:33
			OperatableStateMachine.add('Move above lever',
										_sm_move_above_lever_17,
										transitions={'failed': 'failed', 'continue': 'Read gap pose'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_lever_pose': 'j_lever_pose'})


		# x:984 y:145, x:415 y:316
		_sm_push_or_lever_19 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'true', 'false', 'j_push_pose', 'safe_position_name', 'grab_pos', 'release_pos', 'soft_grab_pos', 'j_above_slider', 'j_slightly_above_slider', 'j_between_slider', 'j_above_vision', 'j_above_holder', 'j_above_table', 'j_init_pose', 'j_init2_pose', 'j_lever_pose', 'action'])

		with _sm_push_or_lever_19:
			# x:66 y:33
			OperatableStateMachine.add('Read HCA TF',
										ReadTFHCA(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_1_link0"),
										transitions={'continue': 'Pick up HCA and put into vise', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})

			# x:277 y:28
			OperatableStateMachine.add('Pick up HCA and put into vise',
										_sm_pick_up_hca_and_put_into_vise_12,
										transitions={'failed': 'failed', 'finished': 'Read action'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos', 'true': 'true', 'false': 'false', 'j_between_slider': 'j_between_slider', 'j_above_vision': 'j_above_vision', 'j_above_holder': 'j_above_holder', 'j_above_table': 'j_above_table', 'j_init_pose': 'j_init_pose'})

			# x:804 y:83
			OperatableStateMachine.add('Push pin out',
										_sm_push_pin_out_3,
										transitions={'failed': 'failed', 'continue': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_push_pose': 'j_push_pose', 'offset': 'offset', 'rotation': 'rotation', 'true': 'true', 'j_init2_pose': 'j_init2_pose', 'false': 'false'})

			# x:541 y:43
			OperatableStateMachine.add('Read action',
										ReadAction(),
										transitions={'move': 'failed', 'cut': 'failed', 'lever': 'Levering action', 'turn_over': 'failed', 'remove_clip': 'Push pin out'},
										autonomy={'move': Autonomy.Off, 'cut': Autonomy.Off, 'lever': Autonomy.Off, 'turn_over': Autonomy.Off, 'remove_clip': Autonomy.Off},
										remapping={'action': 'action', 'action_out': 'action_out'})

			# x:814 y:267
			OperatableStateMachine.add('Levering action',
										_sm_levering_action_18,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'safe_position_name': 'safe_position_name', 'j_lever_pose': 'j_lever_pose', 'j_init2_pose': 'j_init2_pose'})


		# x:250 y:260, x:576 y:264
		_sm_initial_action_20 = OperatableStateMachine(outcomes=['failed', 'finished'], input_keys=['true', 'false', 'offset', 'rotation', 'safe_position_name', 'grab_pos', 'soft_grab_pos', 'release_pos', 'j_push_pose', 'j_above_slider', 'j_slightly_above_slider', 'j_between_slider', 'j_above_vision', 'j_above_holder', 'j_above_table', 'j_init_pose', 'j_init2_pose', 'j_lever_pose'])

		with _sm_initial_action_20:
			# x:150 y:57
			OperatableStateMachine.add('Read recommended action',
										ReadNextVisionAction(),
										transitions={'move': 'failed', 'cut': 'failed', 'lever': 'Push or lever', 'turn_over': 'failed', 'remove_clip': 'Push or lever'},
										autonomy={'move': Autonomy.Off, 'cut': Autonomy.Off, 'lever': Autonomy.Off, 'turn_over': Autonomy.Off, 'remove_clip': Autonomy.Off},
										remapping={'action': 'action'})

			# x:711 y:54
			OperatableStateMachine.add('Push or lever',
										_sm_push_or_lever_19,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'true': 'true', 'false': 'false', 'j_push_pose': 'j_push_pose', 'safe_position_name': 'safe_position_name', 'grab_pos': 'grab_pos', 'release_pos': 'release_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'j_between_slider': 'j_between_slider', 'j_above_vision': 'j_above_vision', 'j_above_holder': 'j_above_holder', 'j_above_table': 'j_above_table', 'j_init_pose': 'j_init_pose', 'j_init2_pose': 'j_init2_pose', 'j_lever_pose': 'j_lever_pose', 'action': 'action'})


		# x:30 y:365, x:130 y:365
		_sm_initialize_holder_and_slider_21 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['false', 'true', 'value'])

		with _sm_initialize_holder_and_slider_21:
			# x:388 y:442
			OperatableStateMachine.add('Open holder',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:406 y:331
			OperatableStateMachine.add('Move slider to front',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Open holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:30 y:51
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'Move slider back', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:426 y:133
			OperatableStateMachine.add('Rotate holder up',
										ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'Wait again', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:476 y:35
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'Rotate holder up'},
										autonomy={'done': Autonomy.Off})

			# x:459 y:234
			OperatableStateMachine.add('Wait again',
										WaitState(wait_time=1),
										transitions={'done': 'Move slider to front'},
										autonomy={'done': Autonomy.Off})

			# x:263 y:41
			OperatableStateMachine.add('Move slider back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_initalize_controllers_2_22 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init2_pose'])

		with _sm_initalize_controllers_2_22:
			# x:54 y:37
			OperatableStateMachine.add('Error recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_2"),
										transitions={'continue': 'Load Cartesian controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:265 y:39
			OperatableStateMachine.add('Load Cartesian controllers',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_2"),
										transitions={'continue': 'Load joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:537 y:33
			OperatableStateMachine.add('Load joint controllers',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_2"),
										transitions={'continue': 'Switch to joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:755 y:231
			OperatableStateMachine.add('Move to initial position',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_2'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:750 y:139
			OperatableStateMachine.add('Read initial joint position',
										ReadFromMongo(),
										transitions={'continue': 'Move to initial position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init2_pose', 'joints_data': 'mdb_init_pose'})

			# x:741 y:34
			OperatableStateMachine.add('Switch to joint controllers',
										SwitchControllerProxyClient(robot_name="panda_2", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'Read initial joint position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_initalize_controllers_23 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init_pose'])

		with _sm_initalize_controllers_23:
			# x:54 y:37
			OperatableStateMachine.add('Error recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'Load Cartesian controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:265 y:39
			OperatableStateMachine.add('Load Cartesian controllers',
										LoadControllerProxyClient(desired_controller=self.cartesian_controller, robot_name="panda_1"),
										transitions={'continue': 'Load joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:537 y:33
			OperatableStateMachine.add('Load joint controllers',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Switch to joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:755 y:231
			OperatableStateMachine.add('Move to initial position',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:750 y:139
			OperatableStateMachine.add('Read initial joint position',
										ReadFromMongo(),
										transitions={'continue': 'Move to initial position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'mdb_init_pose'})

			# x:741 y:34
			OperatableStateMachine.add('Switch to joint controllers',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=[self.cartesian_controller], strictness=1),
										transitions={'continue': 'Read initial joint position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_concurrent_init_24 = ConcurrencyContainer(outcomes=['failed', 'continue'], input_keys=['false', 'true', 'value', 'j_init_pose', 'open_pos', 'j_init2_pose'], conditions=[
										('continue', [('Initialize holder and slider', 'continue'), ('Initalize controllers', 'continue'), ('Initalize controllers_2', 'continue')]),
										('failed', [('Initalize controllers', 'failed'), ('Initialize holder and slider', 'failed'), ('Initalize controllers_2', 'failed')])
										])

		with _sm_concurrent_init_24:
			# x:77 y:42
			OperatableStateMachine.add('Initialize holder and slider',
										_sm_initialize_holder_and_slider_21,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'false': 'false', 'true': 'true', 'value': 'value'})

			# x:325 y:117
			OperatableStateMachine.add('Initalize controllers_2',
										_sm_initalize_controllers_2_22,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init2_pose': 'j_init2_pose'})

			# x:329 y:34
			OperatableStateMachine.add('Initalize controllers',
										_sm_initalize_controllers_23,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose'})


		# x:648 y:224, x:130 y:365
		_sm_cell_initialization_25 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos', 'value', 'false', 'j_init_pose', 'j_init2_pose'])

		with _sm_cell_initialization_25:
			# x:247 y:59
			OperatableStateMachine.add('Concurrent init',
										_sm_concurrent_init_24,
										transitions={'failed': 'failed', 'continue': 'Open SoftHand'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'false': 'false', 'true': 'true', 'value': 'value', 'j_init_pose': 'j_init_pose', 'open_pos': 'open_pos', 'j_init2_pose': 'j_init2_pose'})

			# x:337 y:220
			OperatableStateMachine.add('Open SoftHand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_pos', 'success': 'success'})



		with _state_machine:
			# x:94 y:26
			OperatableStateMachine.add('Cell initialization',
										_sm_cell_initialization_25,
										transitions={'finished': 'Initial action', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos', 'value': 'true', 'false': 'false', 'j_init_pose': 'j_init_pose', 'j_init2_pose': 'j_init2_pose'})

			# x:629 y:119
			OperatableStateMachine.add('Initial action',
										_sm_initial_action_20,
										transitions={'failed': 'failed', 'finished': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'safe_position_name': 'safe_position_name', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'release_pos': 'release_pos', 'j_push_pose': 'j_push_pose', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'j_between_slider': 'j_between_slider', 'j_above_vision': 'j_above_vision', 'j_above_holder': 'j_above_holder', 'j_above_table': 'j_above_table', 'j_init_pose': 'j_init_pose', 'j_init2_pose': 'j_init2_pose', 'j_lever_pose': 'j_lever_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
