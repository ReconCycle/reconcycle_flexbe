#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_states.CallAction_ForceAction import CallForceAction
from reconcycle_flexbe_states.CallAction_JointTrapVel import CallJointTrap
from reconcycle_flexbe_states.CallAction_TF_CartLin import CallActionTFCartLin
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy
from reconcycle_flexbe_states.MoveToJoints import MoveToJoints
from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
from reconcycle_flexbe_states.set_cartesian_compliance_reconcycle import SetReconcycleCartesianCompliance
from reconcycle_flexbe_states.switch_controller_service_client import SwitchControllerProxyClient
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 30 2021
@author: Kubra Karacan, Matija Mavsar, Mihael Simonic
'''
class LeveringActionKalo15HCASM(Behavior):
	'''
	1. Joint move to  levering point
2. Levering action based on Force Action
	'''


	def __init__(self):
		super(LeveringActionKalo15HCASM, self).__init__()
		self.name = 'Levering Action Kalo 1.5 HCA'

		# parameters of this behavior
		self.add_parameter('Kp', 3000)
		self.add_parameter('Kr', 70)
		self.add_parameter('max_vel', 3)
		self.add_parameter('max_acc', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:408 y:559, x:472 y:398
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.j_test = [0.973548,0.50085,-0.535146,-1.80296,-0.517136,1.74385,1.56292]
		_state_machine.userdata.db_pose_vice = 'test/pose_vice'
		_state_machine.userdata.j_init_pose = 'new/init_joints'
		_state_machine.userdata.false = False
		_state_machine.userdata.true = True
		_state_machine.userdata.j_push_pose = 'new/joints/above_push'
		_state_machine.userdata.c_push = 'new/pose/push'
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.rotation = [0,0,0]
		_state_machine.userdata.j_push_1 = 'new/joints/push_1'
		_state_machine.userdata.j_push_2 = 'new/joints/push_2'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_old_init_0 = OperatableStateMachine(outcomes=['failed', 'continue'])

		with _sm_old_init_0:
			# x:39 y:85
			OperatableStateMachine.add('Load controllers',
										LoadControllerProxyClient(desired_controller='joint_impedance_controller', robot_name='panda_1'),
										transitions={'continue': 'Load controllers_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:38 y:172
			OperatableStateMachine.add('Load controllers_2',
										LoadControllerProxyClient(desired_controller='cartesian_impedance_controller_tum', robot_name='panda_1'),
										transitions={'continue': 'Switch to Cartesian', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:311 y:122
			OperatableStateMachine.add('Move to clamp',
										MoveToJoints(joints=[0.973548,0.50085,-0.535146,-1.80296,-0.517136,1.74385,1.56292], max_vel=1, max_acl=1, namespace=''),
										transitions={'continue': 'Move to clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_values': 'joint_values'})

			# x:312 y:40
			OperatableStateMachine.add('Switch to Cartesian',
										SwitchControllerProxyClient(robot_name='panda_1', start_controller=['cartesian_impedance_controller'], stop_controller=['joint_impedance_controller'], strictness=2),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:30 y:260
			OperatableStateMachine.add('Switch to joint controllers',
										SwitchControllerProxyClient(robot_name='', start_controller=['joint_impedance_controller'], stop_controller=['cartesian_impedance_controller_tum'], strictness=2),
										transitions={'continue': 'Move to clamp', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		# x:400 y:313, x:597 y:498
		_sm_push_pin_out_1 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_push_pose', 'offset', 'rotation'])

		with _sm_push_pin_out_1:
			# x:49 y:40
			OperatableStateMachine.add('Read push',
										ReadFromMongo(),
										transitions={'continue': 'move push', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_push_pose', 'joints_data': 'mdb_push_pose'})

			# x:807 y:378
			OperatableStateMachine.add('Move push BACK',
										CallActionTFCartLin(namespace='panda_1', exe_time=3, offset=[0,-0.03,0.004,0,0,0], offset_type='global', limit_rotations=False),
										transitions={'continue': 'continue', 'failed': 'Move push BACK'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})

			# x:839 y:124
			OperatableStateMachine.add('Move push TF',
										CallActionTFCartLin(namespace='panda_1', exe_time=3, offset=[0,0,0.004,0,0,0], offset_type='global', limit_rotations=False),
										transitions={'continue': 'Move push ALL THE WAY', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})

			# x:842 y:41
			OperatableStateMachine.add('Read push TF',
										ReadTFCartLin(target_frame='new/pose/push', source_frame='panda_1/panda_1_link0'),
										transitions={'continue': 'Move push TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_push'})

			# x:217 y:32
			OperatableStateMachine.add('move push',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acc, namespace="panda_1"),
										transitions={'continue': 'switch_on_controller', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_push_pose', 'joint_values': 'joint_values'})

			# x:613 y:32
			OperatableStateMachine.add('set_cart_compliance',
										SetReconcycleCartesianCompliance(robot_name="panda_1", Kp=[self.Kp,self.Kp,self.Kp], Kr=[self.Kr,self.Kr,self.Kr]),
										transitions={'continue': 'Read push TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:428 y:31
			OperatableStateMachine.add('switch_on_controller',
										SwitchControllerProxyClient(robot_name='panda_1', start_controller=["cartesian_impedance_controller"], stop_controller=["joint_impedance_controller"], strictness=2),
										transitions={'continue': 'set_cart_compliance', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:813 y:253
			OperatableStateMachine.add('Move push ALL THE WAY',
										CallActionTFCartLin(namespace='panda_1', exe_time=0.2, offset=[0,0.01,0.004,0,0,0], offset_type='global', limit_rotations=False),
										transitions={'continue': 'Move push BACK', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_push', 't2_out': 't2_out'})


		# x:30 y:365, x:494 y:375
		_sm_init_controllers_and_move_init_2 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init_pose'])

		with _sm_init_controllers_and_move_init_2:
			# x:54 y:32
			OperatableStateMachine.add('Load_cartesian_contr',
										LoadControllerProxyClient(desired_controller="cartesian_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'Load_joint_contr', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:249 y:33
			OperatableStateMachine.add('Load_joint_contr',
										LoadControllerProxyClient(desired_controller="joint_impedance_controller", robot_name="panda_1"),
										transitions={'continue': 'error_recovery', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:861 y:44
			OperatableStateMachine.add('Read init',
										ReadFromMongo(),
										transitions={'continue': 'move init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'entry_name': 'j_init_pose', 'joints_data': 'mdb_init_pose'})

			# x:435 y:30
			OperatableStateMachine.add('error_recovery',
										FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'switch to joint', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:857 y:180
			OperatableStateMachine.add('move init',
										CallJointTrap(max_vel=self.max_vel, max_acl=self.max_acc, namespace="panda_1"),
										transitions={'continue': 'continue', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joints_data': 'mdb_init_pose', 'joint_values': 'joint_values'})

			# x:652 y:31
			OperatableStateMachine.add('switch to joint',
										SwitchControllerProxyClient(robot_name="panda_1", start_controller=["joint_impedance_controller"], stop_controller=["cartesian_impedance_controller"], strictness=2),
										transitions={'continue': 'Read init', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})



		with _state_machine:
			# x:89 y:96
			OperatableStateMachine.add('Init controllers and move init',
										_sm_init_controllers_and_move_init_2,
										transitions={'failed': 'failed', 'continue': 'Push pin out'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose'})

			# x:130 y:283
			OperatableStateMachine.add('Push pin out',
										_sm_push_pin_out_1,
										transitions={'failed': 'failed', 'continue': 'finished'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_push_pose': 'j_push_pose', 'offset': 'offset', 'rotation': 'rotation'})

			# x:993 y:40
			OperatableStateMachine.add('Start force action',
										CallForceAction(direction=[0, 0, 1], amplitude=0.02, frequency=0.5, force=5, rho_min=0.7, namespace='panda_1'),
										transitions={'continue': 'failed', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'success': 'success'})

			# x:815 y:38
			OperatableStateMachine.add('old init',
										_sm_old_init_0,
										transitions={'failed': 'failed', 'continue': 'failed'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
