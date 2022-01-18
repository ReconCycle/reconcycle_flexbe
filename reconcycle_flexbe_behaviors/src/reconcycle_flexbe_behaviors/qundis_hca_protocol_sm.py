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
from reconcycle_flexbe_behaviors.change_tool_on_robot_sm import ChangetoolonrobotSM
from reconcycle_flexbe_behaviors.cutting_pcb_sm import CuttingPCBSM
from reconcycle_flexbe_behaviors.d5_2_sm import D5_2SM
from reconcycle_flexbe_behaviors.pick_plastic_from_clamp_sm import PickplasticfromclampSM
from reconcycle_flexbe_states.CallAction_ForceAction import CallForceAction
from reconcycle_flexbe_states.CallAction_JointTrapVel import CallJointTrap
from reconcycle_flexbe_states.CallAction_TF_CartLin import CallActionTFCartLin
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
Created on @author: Rok Pahic, Boris Kuster, Mihael Simonic, Matija Mavsar
@author: Rok Pahic, Boris Kuster, Mihael Simonic, Matija Mavsar
'''
class QundisHCAprotocolSM(Behavior):
	'''
	Disassembly protocol for the new HCA (Qundis):

- pickup from table using vision
- place into vise
- close the vise
- use levering action to remove PCB
- rotate vise to make PCB fall out
- remove the HCA housing
- move PCB to cutter 
- cut and remove the battery
	'''


	def __init__(self):
		super(QundisHCAprotocolSM, self).__init__()
		self.name = 'Qundis HCA protocol'

		# parameters of this behavior
		self.add_parameter('Kp', 2000)
		self.add_parameter('Kr', 30)
		self.add_parameter('max_acl', 3)
		self.add_parameter('max_vel', 1)

		# references to used behaviors
		self.add_behavior(ChangetoolonrobotSM, 'Change tool and remove HCA housing/Change tool on robot')
		self.add_behavior(PickplasticfromclampSM, 'Change tool and remove HCA housing/Remove housing/Pick plastic from clamp')
		self.add_behavior(D5_2SM, 'D5_2')
		self.add_behavior(CuttingPCBSM, 'Move PCB to cutter and put new HCA into vise/Cutting PCB')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:515 y:387, x:542 y:248
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
		_state_machine.userdata.j_slightly_above_slider = "emo/j_slightly_above_slider_4"
		_state_machine.userdata.rotation = [0,0,0]
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.holder_pickup_pose = "emo/holder_pickup_pose"
		_state_machine.userdata.slight_release_pos = [0.50]
		_state_machine.userdata.soft_grab_pos = [0.7]
		_state_machine.userdata.safe_position_name = "panda_2_safe"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_0:
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


		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_impedance_controller_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_impedance_controller_1:
			# x:536 y:26
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1040 y:110
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:792 y:25
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_place_hca_on_slider_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_place_hca_on_slider_2:
			# x:771 y:38
			OperatableStateMachine.add('read_j_above_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_above_slider'})

			# x:780 y:142
			OperatableStateMachine.add('move_j_above_slider',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'read_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})

			# x:754 y:314
			OperatableStateMachine.add('move_j_slightly_above_slider',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

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
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_robot_above_table_and_get_hca_pose_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'offset', 'rotation'], output_keys=['tf_pickup_pose'])

		with _sm_move_robot_above_table_and_get_hca_pose_3:
			# x:321 y:32
			OperatableStateMachine.add('Wait for motion to stop',
										WaitState(wait_time=1),
										transitions={'done': 'Read object TF'},
										autonomy={'done': Autonomy.Off})

			# x:916 y:240
			OperatableStateMachine.add('Read_above_table',
										ReadFromMongo(),
										transitions={'continue': 'j_move_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'mdb_above_table_pose'})

			# x:930 y:476
			OperatableStateMachine.add('j_move_above_table',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:572 y:17
			OperatableStateMachine.add('Read object TF',
										ReadTFCartLin(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_1_link0"),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})


		# x:30 y:365, x:130 y:365
		_sm_move_above_hca_and_pickup_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos'])

		with _sm_move_above_hca_and_pickup_4:
			# x:408 y:50
			OperatableStateMachine.add('Move pickup above pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=2, offset=[-0.03,0.03,-0.2,-102.472,-3.952,-120], offset_type='local', limit_rotations=True),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:414 y:402
			OperatableStateMachine.add('Move back pickup',
										CallActionTFCartLin(namespace="panda_1", exe_time=2, offset=[-0.03,0.03,-0.2,-102.472,-3.952,-120], offset_type='local', limit_rotations=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:412 y:157
			OperatableStateMachine.add('Move pickup pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=2, offset=[-0.03,0.03,-0.08,-102.472,-3.952,-120], offset_type='local', limit_rotations=True),
										transitions={'continue': 'Grab the object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:418 y:284
			OperatableStateMachine.add('Grab the object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.05),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'soft_grab_pos', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_close_vise_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'value'])

		with _sm_close_vise_5:
			# x:47 y:42
			OperatableStateMachine.add('move_slider_back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:226 y:32
			OperatableStateMachine.add('wait0',
										WaitState(wait_time=1),
										transitions={'done': 'close'},
										autonomy={'done': Autonomy.Off})

			# x:350 y:29
			OperatableStateMachine.add('close',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_pickup_hca_and_put_into_vise_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'offset', 'rotation', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_pickup_hca_and_put_into_vise_6:
			# x:30 y:44
			OperatableStateMachine.add('Move robot above table and get HCA pose',
										_sm_move_robot_above_table_and_get_hca_pose_3,
										transitions={'finished': 'Start Cartesian impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose'})

			# x:624 y:40
			OperatableStateMachine.add('Move above HCA and pickup',
										_sm_move_above_hca_and_pickup_4,
										transitions={'finished': 'Start joint impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos'})

			# x:640 y:231
			OperatableStateMachine.add('Place HCA on slider',
										_sm_place_hca_on_slider_2,
										transitions={'finished': 'Close vise', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:338 y:49
			OperatableStateMachine.add('Start Cartesian impedance controller',
										_sm_start_cartesian_impedance_controller_1,
										transitions={'finished': 'Move above HCA and pickup', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:608 y:137
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_0,
										transitions={'finished': 'Place HCA on slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:648 y:316
			OperatableStateMachine.add('Close vise',
										_sm_close_vise_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'value': 'false'})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_7 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_7:
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


		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_impedance_controller_8 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_impedance_controller_8:
			# x:536 y:26
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1040 y:110
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:792 y:25
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_place_hca_on_slider_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_place_hca_on_slider_9:
			# x:771 y:38
			OperatableStateMachine.add('read_j_above_slider',
										ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_above_slider'})

			# x:780 y:142
			OperatableStateMachine.add('move_j_above_slider',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'read_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})

			# x:754 y:314
			OperatableStateMachine.add('move_j_slightly_above_slider',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

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
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_robot_above_table_and_get_hca_pose_10 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'offset', 'rotation'], output_keys=['tf_pickup_pose'])

		with _sm_move_robot_above_table_and_get_hca_pose_10:
			# x:321 y:32
			OperatableStateMachine.add('Wait for motion to stop',
										WaitState(wait_time=1),
										transitions={'done': 'Read object TF'},
										autonomy={'done': Autonomy.Off})

			# x:916 y:240
			OperatableStateMachine.add('Read_above_table',
										ReadFromMongo(),
										transitions={'continue': 'j_move_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'mdb_above_table_pose'})

			# x:930 y:476
			OperatableStateMachine.add('j_move_above_table',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:572 y:17
			OperatableStateMachine.add('Read object TF',
										ReadTFCartLin(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_1_link0"),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})


		# x:30 y:365, x:130 y:365
		_sm_move_above_hca_and_pickup_11 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos'])

		with _sm_move_above_hca_and_pickup_11:
			# x:408 y:50
			OperatableStateMachine.add('Move pickup above pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=2, offset=[-0.03,0.03,-0.2,-102.472,-3.952,-120], offset_type='local', limit_rotations=True),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:414 y:402
			OperatableStateMachine.add('Move back pickup',
										CallActionTFCartLin(namespace="panda_1", exe_time=2, offset=[-0.03,0.03,-0.2,-102.472,-3.952,-120], offset_type='local', limit_rotations=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:412 y:157
			OperatableStateMachine.add('Move pickup pose',
										CallActionTFCartLin(namespace="panda_1", exe_time=2, offset=[-0.03,0.03,-0.08,-102.472,-3.952,-120], offset_type='local', limit_rotations=True),
										transitions={'continue': 'Grab the object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:418 y:284
			OperatableStateMachine.add('Grab the object',
										MoveSoftHand(motion_duration=2, motion_timestep=0.05),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'soft_grab_pos', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_close_vise_12 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'value'])

		with _sm_close_vise_12:
			# x:47 y:42
			OperatableStateMachine.add('move_slider_back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:226 y:32
			OperatableStateMachine.add('wait0',
										WaitState(wait_time=1),
										transitions={'done': 'close'},
										autonomy={'done': Autonomy.Off})

			# x:350 y:29
			OperatableStateMachine.add('close',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_pickup_hca_and_put_into_vise_2_13 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'offset', 'rotation', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_pickup_hca_and_put_into_vise_2_13:
			# x:30 y:44
			OperatableStateMachine.add('Move robot above table and get HCA pose',
										_sm_move_robot_above_table_and_get_hca_pose_10,
										transitions={'finished': 'Start Cartesian impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose'})

			# x:624 y:40
			OperatableStateMachine.add('Move above HCA and pickup',
										_sm_move_above_hca_and_pickup_11,
										transitions={'finished': 'Start joint impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos'})

			# x:640 y:231
			OperatableStateMachine.add('Place HCA on slider',
										_sm_place_hca_on_slider_9,
										transitions={'finished': 'Close vise', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:338 y:49
			OperatableStateMachine.add('Start Cartesian impedance controller',
										_sm_start_cartesian_impedance_controller_8,
										transitions={'finished': 'Move above HCA and pickup', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:608 y:137
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_7,
										transitions={'finished': 'Place HCA on slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:648 y:316
			OperatableStateMachine.add('Close vise',
										_sm_close_vise_12,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'value': 'false'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_move_pcb_to_cutter_and_put_new_hca_into_vise_14 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'offset', 'rotation', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'], conditions=[
										('finished', [('Pickup HCA and put into vise_2', 'finished'), ('Cutting PCB', 'finished')]),
										('failed', [('Pickup HCA and put into vise_2', 'failed'), ('Cutting PCB', 'failed')])
										])

		with _sm_move_pcb_to_cutter_and_put_new_hca_into_vise_14:
			# x:56 y:40
			OperatableStateMachine.add('Pickup HCA and put into vise_2',
										_sm_pickup_hca_and_put_into_vise_2_13,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:171 y:139
			OperatableStateMachine.add('Cutting PCB',
										self.use_behavior(CuttingPCBSM, 'Move PCB to cutter and put new HCA into vise/Cutting PCB',
											default_keys=['PCB_location_name','simulate_cutter','battery_location_name']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_location_2_15 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_location_2_15:
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


		# x:610 y:284, x:319 y:191
		_sm_levering_action_16 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'safe_position_name'])

		with _sm_levering_action_16:
			# x:80 y:37
			OperatableStateMachine.add('Read gap pose',
										ReadTFCartLin(target_frame="HCA_gap_pose", source_frame="panda_2/panda_2_link0"),
										transitions={'continue': 'Move to gap pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_gap'})

			# x:544 y:171
			OperatableStateMachine.add('Move to safe location_2',
										_sm_move_to_safe_location_2_15,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'safe_position_name'})

			# x:488 y:32
			OperatableStateMachine.add('Start levering',
										CallForceAction(direction=[0,1,0], amplitude=0.02, frequency=3, force=15, rho_min=0.7, namespace='panda_2'),
										transitions={'continue': 'Move to safe location_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'success': 'success'})

			# x:294 y:32
			OperatableStateMachine.add('Move to gap pose',
										CallActionTFCartLin(namespace="panda_2", exe_time=3, offset=0, offset_type='local', limit_rotations=False),
										transitions={'continue': 'Start levering', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_gap', 't2_out': 't2_out'})


		# x:30 y:365, x:130 y:365
		_sm_throw_pcb_from_vise_17 = OperatableStateMachine(outcomes=['continue', 'failed'], input_keys=['FA', 'TR'])

		with _sm_throw_pcb_from_vise_17:
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
										WaitState(wait_time=3.0),
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
		_sm_remove_housing_18 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'])

		with _sm_remove_housing_18:
			# x:65 y:119
			OperatableStateMachine.add('Throw PCB from vise',
										_sm_throw_pcb_from_vise_17,
										transitions={'continue': 'Pick plastic from clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'FA': 'FA', 'TR': 'TR'})

			# x:243 y:238
			OperatableStateMachine.add('Pick plastic from clamp',
										self.use_behavior(PickplasticfromclampSM, 'Change tool and remove HCA housing/Remove housing/Pick plastic from clamp',
											default_keys=['clamp_waiting_location_name','closed_hand_clamp','clamp_pick_location_name','clamp_above_location_name']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_change_tool_and_remove_hca_housing_19 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['true', 'false'], conditions=[
										('finished', [('Change tool on robot', 'finished'), ('Remove housing', 'finished')]),
										('failed', [('Change tool on robot', 'failed'), ('Remove housing', 'failed')])
										])

		with _sm_change_tool_and_remove_hca_housing_19:
			# x:30 y:40
			OperatableStateMachine.add('Change tool on robot',
										self.use_behavior(ChangetoolonrobotSM, 'Change tool and remove HCA housing/Change tool on robot',
											default_keys=['tool_drop_location_name','tool_take_location_name','before_drop_location_name','after_take_location_name','open_air_block']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:307 y:60
			OperatableStateMachine.add('Remove housing',
										_sm_remove_housing_18,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'true', 'FA': 'false'})


		# x:30 y:365, x:162 y:464
		_sm_initialize_holder_and_slider_20 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['false', 'true', 'value'])

		with _sm_initialize_holder_and_slider_20:
			# x:30 y:51
			OperatableStateMachine.add('Open pneumatic block',
										ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'Move slider to front', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:381 y:312
			OperatableStateMachine.add('Move slider to front',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Open holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:378 y:409
			OperatableStateMachine.add('Open holder',
										ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:392 y:130
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

			# x:418 y:225
			OperatableStateMachine.add('Wait again',
										WaitState(wait_time=1),
										transitions={'done': 'Move slider to front'},
										autonomy={'done': Autonomy.Off})

			# x:233 y:34
			OperatableStateMachine.add('Move slider back',
										ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})


		# x:346 y:189, x:314 y:342
		_sm_initalize_controllers_21 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init_pose'])

		with _sm_initalize_controllers_21:
			# x:55 y:38
			OperatableStateMachine.add('Load Cartesian controllers',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Load joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:273 y:37
			OperatableStateMachine.add('Load joint controllers',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Error recovery', 'failed': 'failed'},
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
			OperatableStateMachine.add('Switch to joint cpntrollers',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller"], strictness=1),
										transitions={'continue': 'Read initial joint position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:500 y:35
			OperatableStateMachine.add('Error recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'Switch to joint cpntrollers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:263 y:159, x:230 y:365, x:330 y:365
		_sm_concurrent_init_22 = ConcurrencyContainer(outcomes=['failed', 'continue'], input_keys=['false', 'true', 'value', 'j_init_pose', 'open_pos'], conditions=[
										('continue', [('Initialize holder and slider', 'continue'), ('Initalize controllers', 'continue'), ('Open SoftHand', 'continue')]),
										('failed', [('Initalize controllers', 'failed'), ('Initialize holder and slider', 'failed'), ('Open SoftHand', 'failed')])
										])

		with _sm_concurrent_init_22:
			# x:77 y:42
			OperatableStateMachine.add('Initialize holder and slider',
										_sm_initialize_holder_and_slider_20,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'false': 'false', 'true': 'true', 'value': 'value'})

			# x:337 y:220
			OperatableStateMachine.add('Open SoftHand',
										MoveSoftHand(motion_duration=2, motion_timestep=0.1),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'open_pos', 'success': 'success'})

			# x:329 y:34
			OperatableStateMachine.add('Initalize controllers',
										_sm_initalize_controllers_21,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose'})


		# x:560 y:111, x:562 y:500
		_sm_cell_initialization_23 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos', 'value', 'false', 'j_init_pose'])

		with _sm_cell_initialization_23:
			# x:247 y:59
			OperatableStateMachine.add('Concurrent init',
										_sm_concurrent_init_22,
										transitions={'failed': 'failed', 'continue': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'false': 'false', 'true': 'true', 'value': 'value', 'j_init_pose': 'j_init_pose', 'open_pos': 'open_pos'})



		with _state_machine:
			# x:94 y:26
			OperatableStateMachine.add('Cell initialization',
										_sm_cell_initialization_23,
										transitions={'finished': 'Pickup HCA and put into vise', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos', 'value': 'true', 'false': 'false', 'j_init_pose': 'j_init_pose'})

			# x:748 y:167
			OperatableStateMachine.add('Change tool and remove HCA housing',
										_sm_change_tool_and_remove_hca_housing_19,
										transitions={'finished': 'Move PCB to cutter and put new HCA into vise', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})

			# x:57 y:618
			OperatableStateMachine.add('D5_2',
										self.use_behavior(D5_2SM, 'D5_2'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:796 y:37
			OperatableStateMachine.add('Levering action',
										_sm_levering_action_16,
										transitions={'finished': 'Change tool and remove HCA housing', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'safe_position_name': 'safe_position_name'})

			# x:717 y:322
			OperatableStateMachine.add('Move PCB to cutter and put new HCA into vise',
										_sm_move_pcb_to_cutter_and_put_new_hca_into_vise_14,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:444 y:31
			OperatableStateMachine.add('Pickup HCA and put into vise',
										_sm_pickup_hca_and_put_into_vise_6,
										transitions={'finished': 'Levering action', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
