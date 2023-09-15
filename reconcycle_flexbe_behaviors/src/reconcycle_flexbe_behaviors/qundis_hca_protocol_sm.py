#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState as flexbe_states__WaitState
from rbs_flexbe_states.RBS_JMove import RBS_JMove as rbs_flexbe_states__RBS_JMove
from rbs_flexbe_states.RBS_error_recovery import RBS_FrankaErrorRecoveryActionProxy as rbs_flexbe_states__RBS_FrankaErrorRecoveryActionProxy
from rbs_flexbe_states.RBS_load_controller import RBS_LoadController as rbs_flexbe_states__RBS_LoadController
from rbs_flexbe_states.RBS_switch_controller import RBS_SwitchController as rbs_flexbe_states__RBS_SwitchController
from rbs_flexbe_states.inst_robotblockset import Instantiate_robotblockset as rbs_flexbe_states__Instantiate_robotblockset
from reconcycle_flexbe_behaviors.change_tool_on_robot_sm import ChangetoolonrobotSM
from reconcycle_flexbe_behaviors.cutting_pcb_sm import CuttingPCBSM
from reconcycle_flexbe_behaviors.pick_plastic_from_clamp_sm import PickplasticfromclampSM
from reconcycle_flexbe_states.CallAction_JointTrapVel import CallJointTrap as reconcycle_flexbe_states__CallJointTrap
from reconcycle_flexbe_states.CallAction_TF_CartLin import CallActionTFCartLin as reconcycle_flexbe_states__CallActionTFCartLin
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy as reconcycle_flexbe_states__FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.MoveSoftHand import MoveSoftHand as reconcycle_flexbe_states__MoveSoftHand
from reconcycle_flexbe_states.ReadNextVisionAction import ReadNextVisionAction as reconcycle_flexbe_states__ReadNextVisionAction
from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin as reconcycle_flexbe_states__ReadTFCartLin
from reconcycle_flexbe_states.active_controller_service_client import ActiveControllerProxyClient as reconcycle_flexbe_states__ActiveControllerProxyClient
from reconcycle_flexbe_states.avtivate_raspi_output import ActivateRaspiDigitalOuput as reconcycle_flexbe_states__ActivateRaspiDigitalOuput
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient as reconcycle_flexbe_states__LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo as reconcycle_flexbe_states__ReadFromMongo
from reconcycle_flexbe_states.set_cartesian_compliance_reconcycle import SetReconcycleCartesianCompliance as reconcycle_flexbe_states__SetReconcycleCartesianCompliance
from reconcycle_flexbe_states.switch_controller_service_client import SwitchControllerProxyClient as reconcycle_flexbe_states__SwitchControllerProxyClient
from reconcycle_flexbe_states.unload_controller_service_client import UnloadControllerProxyClient as reconcycle_flexbe_states__UnloadControllerProxyClient
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
		self.add_parameter('namespace', '')
		self.add_parameter('tray_service_name', '')

		# references to used behaviors
		self.add_behavior(ChangetoolonrobotSM, 'Action selection/Change tool and remove HCA housing/Change tool on robot')
		self.add_behavior(PickplasticfromclampSM, 'Action selection/Change tool and remove HCA housing/Remove housing/Pick plastic from clamp')
		self.add_behavior(CuttingPCBSM, 'Action selection/Move PCB to cutter and put new HCA into vise/Cutting PCB')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:515 y:387, x:495 y:284
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
		_state_machine.userdata.j_push_pose = "new/joints/above_push"
		_state_machine.userdata.panda_1 = "panda_1"
		_state_machine.userdata.panda_2 = "panda_2"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:162 y:464
		_sm_initialize_holder_and_slider_0 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['false', 'true', 'value'])

		with _sm_initialize_holder_and_slider_0:
			# x:30 y:51
			OperatableStateMachine.add('Open pneumatic block',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/obr_block2_ON"),
										transitions={'continue': 'Move slider back', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})

			# x:410 y:338
			OperatableStateMachine.add('Move slider to front',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Open holder', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:388 y:442
			OperatableStateMachine.add('Open holder',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:426 y:133
			OperatableStateMachine.add('Rotate holder up',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/obr_rotate"),
										transitions={'continue': 'Wait again', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})

			# x:476 y:35
			OperatableStateMachine.add('Wait',
										flexbe_states__WaitState(wait_time=1),
										transitions={'done': 'Rotate holder up'},
										autonomy={'done': Autonomy.Off})

			# x:459 y:234
			OperatableStateMachine.add('Wait again',
										flexbe_states__WaitState(wait_time=1),
										transitions={'done': 'Move slider to front'},
										autonomy={'done': Autonomy.Off})

			# x:263 y:41
			OperatableStateMachine.add('Move slider back',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'Wait', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'false', 'success': 'success'})


		# x:321 y:507, x:872 y:535
		_sm_initalize_controllers_1 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init_pose'])

		with _sm_initalize_controllers_1:
			# x:14 y:258
			OperatableStateMachine.add('Inst panda_1',
										rbs_flexbe_states__Instantiate_robotblockset(robot_namespace="panda_1"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:438 y:147
			OperatableStateMachine.add('Error recoveryss',
										rbs_flexbe_states__RBS_FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'Switch to joint contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:66 y:156
			OperatableStateMachine.add('Load Cart controllers',
										rbs_flexbe_states__RBS_LoadController(robot_name="panda_1", desired_controller="cartesian_impedance_controller_tum"),
										transitions={'continue': 'Load Joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:55 y:38
			OperatableStateMachine.add('Load Cartesian controllers',
										reconcycle_flexbe_states__LoadControllerProxyClient(desired_controller="cartesian_impedance_controller_tum", robot_name="panda_1"),
										transitions={'continue': 'Load joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:258 y:172
			OperatableStateMachine.add('Load Joint controllers',
										rbs_flexbe_states__RBS_LoadController(robot_name="panda_1", desired_controller="joint_impedance_controller"),
										transitions={'continue': 'Error recoveryss', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:273 y:37
			OperatableStateMachine.add('Load joint controllers',
										reconcycle_flexbe_states__LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Error recovery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:903 y:228
			OperatableStateMachine.add('Move to init',
										rbs_flexbe_states__RBS_JMove(robot_name="panda_1", q=[None,0,0,0,0,0,0], t=5, traj='poly'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'joints_data', 'panda_1': 'panda_1', 'panda_2': 'panda_2', 't2_out': 't2_out'})

			# x:1105 y:88
			OperatableStateMachine.add('Move to initial position',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:911 y:139
			OperatableStateMachine.add('Read initial joint position',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'Move to init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'joints_data'})

			# x:700 y:144
			OperatableStateMachine.add('Switch to joint contr',
										rbs_flexbe_states__RBS_SwitchController(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller_tum"], strictness=1),
										transitions={'continue': 'Move to init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:741 y:34
			OperatableStateMachine.add('Switch to joint controllers',
										reconcycle_flexbe_states__SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller_tum"], strictness=1),
										transitions={'continue': 'Read initial joint position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:500 y:35
			OperatableStateMachine.add('Error recovery',
										reconcycle_flexbe_states__FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'Switch to joint controllers', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:263 y:159, x:230 y:365, x:330 y:365
		_sm_concurrent_init_2 = ConcurrencyContainer(outcomes=['failed', 'continue'], input_keys=['false', 'true', 'value', 'j_init_pose', 'open_pos'], conditions=[
										('continue', [('Initialize holder and slider', 'continue'), ('Initalize controllers', 'continue')]),
										('failed', [('Initalize controllers', 'failed'), ('Initialize holder and slider', 'failed')])
										])

		with _sm_concurrent_init_2:
			# x:77 y:42
			OperatableStateMachine.add('Initialize holder and slider',
										_sm_initialize_holder_and_slider_0,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'false': 'false', 'true': 'true', 'value': 'value'})

			# x:329 y:34
			OperatableStateMachine.add('Initalize controllers',
										_sm_initalize_controllers_1,
										transitions={'failed': 'failed', 'continue': 'continue'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose'})


		# x:560 y:111, x:562 y:500
		_sm_cell_initialization_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'open_pos', 'value', 'false', 'j_init_pose'])

		with _sm_cell_initialization_3:
			# x:247 y:59
			OperatableStateMachine.add('Concurrent init',
										_sm_concurrent_init_2,
										transitions={'failed': 'failed', 'continue': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'false': 'false', 'true': 'true', 'value': 'value', 'j_init_pose': 'j_init_pose', 'open_pos': 'open_pos'})


		# x:30 y:365, x:130 y:365
		_sm_push_pin_out_4 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_push_pose', 'offset', 'rotation'])

		with _sm_push_pin_out_4:
			# x:49 y:40
			OperatableStateMachine.add('Read push',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'move push', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_push_pose', 'joints_data': 'mdb_push_pose'})

			# x:807 y:378
			OperatableStateMachine.add('Move push BACK',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace='panda_1', exe_time=3, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'continue', 'failed': 'Move push BACK'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})

			# x:839 y:124
			OperatableStateMachine.add('Move push TF',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace='panda_1', exe_time=3, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Move push ALL THE WAY', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})

			# x:842 y:41
			OperatableStateMachine.add('Read push TF',
										reconcycle_flexbe_states__ReadTFCartLin(target_frame='new/pose/push', source_frame='panda_1/panda_1_link0'),
										transitions={'continue': 'Move push TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_push'})

			# x:217 y:32
			OperatableStateMachine.add('move push',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_push_pose', 'joint_values': 'joint_values'})

			# x:613 y:32
			OperatableStateMachine.add('set_cart_compliance',
										reconcycle_flexbe_states__SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'Read push TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:428 y:31
			OperatableStateMachine.add('switch_on_controller',
										reconcycle_flexbe_states__SwitchControllerProxyClient(robot_name='panda_1', start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=2),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:813 y:253
			OperatableStateMachine.add('Move push ALL THE WAY',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace='panda_1', exe_time=0.2, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Move push BACK', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})


		# x:30 y:365, x:130 y:365
		_sm_start_joint_impedance_controller_5 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_5:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										reconcycle_flexbe_states__SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller_tum"], strictness=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:81 y:195
			OperatableStateMachine.add('load_joint_controller',
										reconcycle_flexbe_states__LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:437 y:336
			OperatableStateMachine.add('unload_cartesian_controller',
										reconcycle_flexbe_states__UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:73 y:108
			OperatableStateMachine.add('find_active_controller',
										reconcycle_flexbe_states__ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:130 y:365
		_sm_place_hca_on_slider_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_place_hca_on_slider_6:
			# x:771 y:38
			OperatableStateMachine.add('read_j_above_slider',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_above_slider'})

			# x:780 y:142
			OperatableStateMachine.add('move_j_above_slider',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'read_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})

			# x:754 y:314
			OperatableStateMachine.add('move_j_slightly_above_slider',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:753 y:226
			OperatableStateMachine.add('read_j_slightly_above_slider',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'move_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_slightly_above_slider', 'joints_data': 'joints_data'})

			# x:773 y:407
			OperatableStateMachine.add('release object',
										reconcycle_flexbe_states__MoveSoftHand(motion_duration=1, motion_timestep=0.02),
										transitions={'continue': 'move_back_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'release_pos', 'success': 'success'})

			# x:773 y:484
			OperatableStateMachine.add('move_back_above_slider',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_above_hca_and_pickup_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos'])

		with _sm_move_above_hca_and_pickup_7:
			# x:408 y:50
			OperatableStateMachine.add('Move pickup above pose',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=0, limit_rotations=True),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:414 y:402
			OperatableStateMachine.add('Move back pickup',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=0, limit_rotations=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:412 y:157
			OperatableStateMachine.add('Move pickup pose',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=0, limit_rotations=True),
										transitions={'continue': 'Grab the object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:418 y:284
			OperatableStateMachine.add('Grab the object',
										reconcycle_flexbe_states__MoveSoftHand(motion_duration=2, motion_timestep=0.05),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'soft_grab_pos', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_close_vise_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'value'])

		with _sm_close_vise_8:
			# x:47 y:42
			OperatableStateMachine.add('move_slider_back',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:226 y:32
			OperatableStateMachine.add('wait0',
										flexbe_states__WaitState(wait_time=1),
										transitions={'done': 'close'},
										autonomy={'done': Autonomy.Off})

			# x:350 y:29
			OperatableStateMachine.add('close',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_pick_up_hca_and_put_into_vise_9 = OperatableStateMachine(outcomes=['failed', 'finished'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos', 'true', 'false'])

		with _sm_pick_up_hca_and_put_into_vise_9:
			# x:46 y:40
			OperatableStateMachine.add('Move above HCA and pickup',
										_sm_move_above_hca_and_pickup_7,
										transitions={'finished': 'Start joint impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos'})

			# x:62 y:231
			OperatableStateMachine.add('Place HCA on slider',
										_sm_place_hca_on_slider_6,
										transitions={'finished': 'Close vise', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:30 y:137
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_5,
										transitions={'finished': 'Place HCA on slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:70 y:316
			OperatableStateMachine.add('Close vise',
										_sm_close_vise_8,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'value': 'false'})


		# x:30 y:365, x:130 y:365
		_sm_move_above_table_10 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table'])

		with _sm_move_above_table_10:
			# x:30 y:40
			OperatableStateMachine.add('Read_above_table',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'j_move_above_table', 'failed': 'j_move_above_table'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'mdb_above_table_pose'})

			# x:46 y:160
			OperatableStateMachine.add('j_move_above_table',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:388
		_sm_start_joint_impedance_controller_11 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_joint_impedance_controller_11:
			# x:511 y:54
			OperatableStateMachine.add('switch_on_controller',
										reconcycle_flexbe_states__SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller_tum"], strictness=1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:728 y:134
			OperatableStateMachine.add('load_joint_controller',
										reconcycle_flexbe_states__LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:540 y:311
			OperatableStateMachine.add('unload_cartesian_controller',
										reconcycle_flexbe_states__UnloadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:483 y:213
			OperatableStateMachine.add('find_active_controller',
										reconcycle_flexbe_states__ActiveControllerProxyClient(robot_name="panda_1", real_controllers="cartesian_impedance_controller"),
										transitions={'continue': 'load_joint_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'active_controller': 'active_controller'})


		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_impedance_controller_12 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_impedance_controller_12:
			# x:536 y:26
			OperatableStateMachine.add('switch_on_controller',
										reconcycle_flexbe_states__SwitchControllerProxyClient(robot_name="panda_1", start_controller=["cartesian_impedance_controller_tum"], stop_controller=["joint_impedance_controller"], strictness=1),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1040 y:110
			OperatableStateMachine.add('wait1',
										flexbe_states__WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:792 y:25
			OperatableStateMachine.add('set_cart_compliance',
										reconcycle_flexbe_states__SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'wait1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_place_hca_on_slider_13 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_place_hca_on_slider_13:
			# x:771 y:38
			OperatableStateMachine.add('read_j_above_slider',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'move_j_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_slider', 'joints_data': 'joints_above_slider'})

			# x:780 y:142
			OperatableStateMachine.add('move_j_above_slider',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'read_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})

			# x:754 y:314
			OperatableStateMachine.add('move_j_slightly_above_slider',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'release object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_data', 'joint_values': 'joint_values'})

			# x:753 y:226
			OperatableStateMachine.add('read_j_slightly_above_slider',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'move_j_slightly_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_slightly_above_slider', 'joints_data': 'joints_data'})

			# x:773 y:407
			OperatableStateMachine.add('release object',
										reconcycle_flexbe_states__MoveSoftHand(motion_duration=1, motion_timestep=0.02),
										transitions={'continue': 'move_back_above_slider', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'release_pos', 'success': 'success'})

			# x:773 y:484
			OperatableStateMachine.add('move_back_above_slider',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace='panda_1'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'joints_above_slider', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_move_robot_above_table_and_get_hca_pose_14 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['j_above_table', 'offset', 'rotation'], output_keys=['tf_pickup_pose'])

		with _sm_move_robot_above_table_and_get_hca_pose_14:
			# x:321 y:32
			OperatableStateMachine.add('Wait for motion to stop',
										flexbe_states__WaitState(wait_time=1),
										transitions={'done': 'Read object TF'},
										autonomy={'done': Autonomy.Off})

			# x:916 y:240
			OperatableStateMachine.add('Read_above_table',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'j_move_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_above_table', 'joints_data': 'mdb_above_table_pose'})

			# x:930 y:476
			OperatableStateMachine.add('j_move_above_table',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace="panda_1"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_above_table_pose', 'joint_values': 'joint_values'})

			# x:622 y:95
			OperatableStateMachine.add('Read object TF',
										reconcycle_flexbe_states__ReadTFCartLin(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_1_link0"),
										transitions={'continue': 'Read_above_table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})


		# x:30 y:365, x:130 y:365
		_sm_move_above_hca_and_pickup_15 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'tf_pickup_pose', 'grab_pos', 'soft_grab_pos'])

		with _sm_move_above_hca_and_pickup_15:
			# x:408 y:50
			OperatableStateMachine.add('Move pickup above pose',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=0, limit_rotations=True),
										transitions={'continue': 'Move pickup pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:414 y:402
			OperatableStateMachine.add('Move back pickup',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=0, limit_rotations=True),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:412 y:157
			OperatableStateMachine.add('Move pickup pose',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_1", exe_time=2, local_offset=0, global_pos_offset=0, limit_rotations=True),
										transitions={'continue': 'Grab the object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_pickup_pose', 't2_out': 't2_out'})

			# x:418 y:284
			OperatableStateMachine.add('Grab the object',
										reconcycle_flexbe_states__MoveSoftHand(motion_duration=2, motion_timestep=0.05),
										transitions={'continue': 'Move back pickup', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'soft_grab_pos', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_close_vise_16 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'value'])

		with _sm_close_vise_16:
			# x:47 y:42
			OperatableStateMachine.add('move_slider_back',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/move_slide"),
										transitions={'continue': 'wait0', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'value', 'success': 'success'})

			# x:226 y:32
			OperatableStateMachine.add('wait0',
										flexbe_states__WaitState(wait_time=1),
										transitions={'done': 'close'},
										autonomy={'done': Autonomy.Off})

			# x:350 y:29
			OperatableStateMachine.add('close',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name="/obr_activate"),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'value': 'true', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_pickup_hca_and_put_into_vise_2_17 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'offset', 'rotation', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'])

		with _sm_pickup_hca_and_put_into_vise_2_17:
			# x:30 y:44
			OperatableStateMachine.add('Move robot above table and get HCA pose',
										_sm_move_robot_above_table_and_get_hca_pose_14,
										transitions={'finished': 'Start Cartesian impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose'})

			# x:624 y:40
			OperatableStateMachine.add('Move above HCA and pickup',
										_sm_move_above_hca_and_pickup_15,
										transitions={'finished': 'Start joint impedance controller', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos'})

			# x:640 y:231
			OperatableStateMachine.add('Place HCA on slider',
										_sm_place_hca_on_slider_13,
										transitions={'finished': 'Close vise', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:338 y:49
			OperatableStateMachine.add('Start Cartesian impedance controller',
										_sm_start_cartesian_impedance_controller_12,
										transitions={'finished': 'Move above HCA and pickup', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:608 y:137
			OperatableStateMachine.add('Start joint impedance controller',
										_sm_start_joint_impedance_controller_11,
										transitions={'finished': 'Place HCA on slider', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:648 y:316
			OperatableStateMachine.add('Close vise',
										_sm_close_vise_16,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'value': 'false'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_move_pcb_to_cutter_and_put_new_hca_into_vise_18 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['true', 'false', 'offset', 'rotation', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos'], conditions=[
										('finished', [('Pickup HCA and put into vise_2', 'finished'), ('Cutting PCB', 'finished')]),
										('failed', [('Pickup HCA and put into vise_2', 'failed'), ('Cutting PCB', 'failed')])
										])

		with _sm_move_pcb_to_cutter_and_put_new_hca_into_vise_18:
			# x:56 y:40
			OperatableStateMachine.add('Pickup HCA and put into vise_2',
										_sm_pickup_hca_and_put_into_vise_2_17,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:171 y:139
			OperatableStateMachine.add('Cutting PCB',
										self.use_behavior(CuttingPCBSM, 'Action selection/Move PCB to cutter and put new HCA into vise/Cutting PCB',
											default_keys=['PCB_location_name','simulate_cutter','battery_location_name']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_location_2_19 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_location_2_19:
			# x:182 y:50
			OperatableStateMachine.add('Read robot position',
										reconcycle_flexbe_states__ReadFromMongo(),
										transitions={'continue': 'Move to robot position', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'position_name', 'joints_data': 'joints_positions'})

			# x:500 y:117
			OperatableStateMachine.add('Move to robot position',
										reconcycle_flexbe_states__CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acl, namespace=self.namespace),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joints_data': 'joints_positions', 'joint_values': 'joint_values'})


		# x:30 y:365, x:130 y:365
		_sm_levering_action_20 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offset', 'rotation', 'safe_position_name'])

		with _sm_levering_action_20:
			# x:80 y:37
			OperatableStateMachine.add('Read gap pose',
										reconcycle_flexbe_states__ReadTFCartLin(target_frame="HCA_gap_pose", source_frame="panda_2/panda_2_link0"),
										transitions={'continue': 'Move to gap pose', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_gap'})

			# x:544 y:171
			OperatableStateMachine.add('Move to safe location_2',
										_sm_move_to_safe_location_2_19,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'safe_position_name'})

			# x:294 y:32
			OperatableStateMachine.add('Move to gap pose',
										reconcycle_flexbe_states__CallActionTFCartLin(namespace="panda_2", exe_time=3, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'Move to safe location_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_gap', 't2_out': 't2_out'})


		# x:30 y:365, x:130 y:365
		_sm_throw_pcb_from_vise_21 = OperatableStateMachine(outcomes=['continue', 'failed'], input_keys=['FA', 'TR'])

		with _sm_throw_pcb_from_vise_21:
			# x:284 y:40
			OperatableStateMachine.add('Pull in tray',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name=self.tray_service_name),
										transitions={'continue': 'Wait to get back ', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:491 y:426
			OperatableStateMachine.add('Push out tray',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name=self.tray_service_name),
										transitions={'continue': 'Open clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'TR', 'success': 'success'})

			# x:673 y:32
			OperatableStateMachine.add('Rotate CLAMP down',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name='/obr_rotate'),
										transitions={'continue': 'Wait to PCB fall', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'value': 'TR', 'success': 'success'})

			# x:818 y:260
			OperatableStateMachine.add('Rotate CLAMP up',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name='/obr_rotate'),
										transitions={'continue': 'Wait to get back _2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})

			# x:858 y:37
			OperatableStateMachine.add('Wait to PCB fall',
										flexbe_states__WaitState(wait_time=3.0),
										transitions={'done': 'Rotate CLAMP up'},
										autonomy={'done': Autonomy.Low})

			# x:497 y:28
			OperatableStateMachine.add('Wait to get back ',
										flexbe_states__WaitState(wait_time=3.0),
										transitions={'done': 'Rotate CLAMP down'},
										autonomy={'done': Autonomy.Low})

			# x:694 y:382
			OperatableStateMachine.add('Wait to get back _2',
										flexbe_states__WaitState(wait_time=3.0),
										transitions={'done': 'Push out tray'},
										autonomy={'done': Autonomy.Low})

			# x:294 y:491
			OperatableStateMachine.add('Open clamp',
										reconcycle_flexbe_states__ActivateRaspiDigitalOuput(service_name='/obr_activate'),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'FA', 'success': 'success'})


		# x:30 y:365, x:130 y:365
		_sm_remove_housing_22 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['TR', 'FA'])

		with _sm_remove_housing_22:
			# x:65 y:119
			OperatableStateMachine.add('Throw PCB from vise',
										_sm_throw_pcb_from_vise_21,
										transitions={'continue': 'Pick plastic from clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'FA': 'FA', 'TR': 'TR'})

			# x:243 y:238
			OperatableStateMachine.add('Pick plastic from clamp',
										self.use_behavior(PickplasticfromclampSM, 'Action selection/Change tool and remove HCA housing/Remove housing/Pick plastic from clamp',
											default_keys=['clamp_waiting_location_name','closed_hand_clamp','clamp_pick_location_name','clamp_above_location_name']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_change_tool_and_remove_hca_housing_23 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['true', 'false'], conditions=[
										('finished', [('Change tool on robot', 'finished'), ('Remove housing', 'finished')]),
										('failed', [('Change tool on robot', 'failed'), ('Remove housing', 'failed')])
										])

		with _sm_change_tool_and_remove_hca_housing_23:
			# x:30 y:40
			OperatableStateMachine.add('Change tool on robot',
										self.use_behavior(ChangetoolonrobotSM, 'Action selection/Change tool and remove HCA housing/Change tool on robot',
											default_keys=['tool_drop_location_name','tool_take_location_name','before_drop_location_name','after_take_location_name','open_air_block']),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:307 y:60
			OperatableStateMachine.add('Remove housing',
										_sm_remove_housing_22,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'TR': 'true', 'FA': 'false'})


		# x:65 y:200, x:786 y:142
		_sm_action_selection_24 = OperatableStateMachine(outcomes=['failed', 'finished'], input_keys=['true', 'false', 'offset', 'rotation', 'safe_position_name', 'grab_pos', 'soft_grab_pos', 'j_above_table', 'j_between_slider', 'j_above_slider', 'j_slightly_above_slider', 'release_pos', 'j_push_pose'])

		with _sm_action_selection_24:
			# x:159 y:28
			OperatableStateMachine.add('Read recommended action',
										reconcycle_flexbe_states__ReadNextVisionAction(),
										transitions={'move': 'failed', 'cut': 'Move PCB to cutter and put new HCA into vise', 'lever': 'Levering action', 'turn_over': 'failed', 'remove_clip': 'Push pin out'},
										autonomy={'move': Autonomy.Off, 'cut': Autonomy.Off, 'lever': Autonomy.Off, 'turn_over': Autonomy.Off, 'remove_clip': Autonomy.Off},
										remapping={'action': 'action'})

			# x:517 y:209
			OperatableStateMachine.add('Levering action',
										_sm_levering_action_20,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'safe_position_name': 'safe_position_name'})

			# x:367 y:393
			OperatableStateMachine.add('Move PCB to cutter and put new HCA into vise',
										_sm_move_pcb_to_cutter_and_put_new_hca_into_vise_18,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos'})

			# x:485 y:487
			OperatableStateMachine.add('Move above table',
										_sm_move_above_table_10,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'j_above_table': 'j_above_table'})

			# x:435 y:126
			OperatableStateMachine.add('Pick up HCA and put into vise',
										_sm_pick_up_hca_and_put_into_vise_9,
										transitions={'failed': 'failed', 'finished': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'offset': 'offset', 'rotation': 'rotation', 'tf_pickup_pose': 'tf_pickup_pose', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos', 'true': 'true', 'false': 'false'})

			# x:519 y:37
			OperatableStateMachine.add('Push pin out',
										_sm_push_pin_out_4,
										transitions={'failed': 'failed', 'continue': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_push_pose': 'j_push_pose', 'offset': 'offset', 'rotation': 'rotation'})

			# x:407 y:302
			OperatableStateMachine.add('Change tool and remove HCA housing',
										_sm_change_tool_and_remove_hca_housing_23,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false'})



		with _state_machine:
			# x:143 y:19
			OperatableStateMachine.add('Cell initialization',
										_sm_cell_initialization_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'true': 'true', 'open_pos': 'release_pos', 'value': 'true', 'false': 'false', 'j_init_pose': 'j_init_pose'})

			# x:390 y:19
			OperatableStateMachine.add('Read object TF',
										reconcycle_flexbe_states__ReadTFCartLin(target_frame="hca_back_vision_table_zero", source_frame="panda_1/panda_1_link0"),
										transitions={'continue': 'Action selection', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_pickup_pose'})

			# x:742 y:199
			OperatableStateMachine.add('Action selection',
										_sm_action_selection_24,
										transitions={'failed': 'failed', 'finished': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
										remapping={'true': 'true', 'false': 'false', 'offset': 'offset', 'rotation': 'rotation', 'safe_position_name': 'safe_position_name', 'grab_pos': 'grab_pos', 'soft_grab_pos': 'soft_grab_pos', 'j_above_table': 'j_above_table', 'j_between_slider': 'j_between_slider', 'j_above_slider': 'j_above_slider', 'j_slightly_above_slider': 'j_slightly_above_slider', 'release_pos': 'release_pos', 'j_push_pose': 'j_push_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
