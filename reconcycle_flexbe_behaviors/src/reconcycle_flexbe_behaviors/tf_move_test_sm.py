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
from reconcycle_flexbe_states.Read_TF_CartLin import ReadTFCartLin
from reconcycle_flexbe_states.load_controller_service_client import LoadControllerProxyClient
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
from reconcycle_flexbe_states.set_cartesian_compliance_reconcycle import SetReconcycleCartesianCompliance
from reconcycle_flexbe_states.switch_controller_service_client import SwitchControllerProxyClient
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jan 12 2022
@author: Matija Mavsar
'''
class TF_move_testSM(Behavior):
	'''
	Behavior for testing the moving of robot to frames, saved into MongoDB.
	'''


	def __init__(self):
		super(TF_move_testSM, self).__init__()
		self.name = 'TF_move_test'

		# parameters of this behavior
		self.add_parameter('max_vel', 3)
		self.add_parameter('max_acc', 3)
		self.add_parameter('Kp', 2000)
		self.add_parameter('Kr', 30)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:812 y:254, x:524 y:206
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.rotation = [0,0,0]
		_state_machine.userdata.j_init_pose = 'new/init_joints'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_start_cartesian_imp_controller_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_start_cartesian_imp_controller_0:
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
		_sm_init_controllers_and_move_init_1 = OperatableStateMachine(outcomes=['failed', 'continue'], input_keys=['j_init_pose'])

		with _sm_init_controllers_and_move_init_1:
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
			# x:89 y:219
			OperatableStateMachine.add('Init controllers and move init',
										_sm_init_controllers_and_move_init_1,
										transitions={'failed': 'failed', 'continue': 'start_cartesian_imp_controller'},
										autonomy={'failed': Autonomy.Inherit, 'continue': Autonomy.Inherit},
										remapping={'j_init_pose': 'j_init_pose'})

			# x:747 y:46
			OperatableStateMachine.add('Move to TF',
										CallActionTFCartLin(namespace='panda_1', exe_time=6, offset=[-0.18,0,-0.3,0,0,180], offset_type='local', limit_rotations=False),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 'tf_test', 't2_out': 't2_out'})

			# x:544 y:41
			OperatableStateMachine.add('Read TF',
										ReadTFCartLin(target_frame='hca_back_vision_table_zero', source_frame='panda_1/panda_1_link0'),
										transitions={'continue': 'Move to TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'offset': 'offset', 'rotation': 'rotation', 't2_data': 'tf_test'})

			# x:91 y:44
			OperatableStateMachine.add('start_cartesian_imp_controller',
										_sm_start_cartesian_imp_controller_0,
										transitions={'finished': 'Error recovery', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:317 y:45
			OperatableStateMachine.add('Error recovery',
										FrankaErrorRecoveryActionProxy(robot_name='panda_1'),
										transitions={'continue': 'Read TF', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
