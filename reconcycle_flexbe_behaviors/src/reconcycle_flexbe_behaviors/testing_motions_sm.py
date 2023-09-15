#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from rbs_flexbe_states.RBS_CMove import RBS_CMove as rbs_flexbe_states__RBS_CMove
from rbs_flexbe_states.RBS_CMoveFor import RBS_CMoveFor as rbs_flexbe_states__RBS_CMoveFor
from rbs_flexbe_states.RBS_JMove import RBS_JMove as rbs_flexbe_states__RBS_JMove
from rbs_flexbe_states.RBS_error_recovery import RBS_FrankaErrorRecoveryActionProxy as rbs_flexbe_states__RBS_FrankaErrorRecoveryActionProxy
from rbs_flexbe_states.inst_robotblockset import Instantiate_robotblockset as rbs_flexbe_states__Instantiate_robotblockset
from reconcycle_flexbe_states.ErrorRecoveryProxy import FrankaErrorRecoveryActionProxy as reconcycle_flexbe_states__FrankaErrorRecoveryActionProxy
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Sep 13 2021
@author: miha
'''
class testing_motionsSM(Behavior):
	'''
	Testing cartesian and joint controllers.
	'''


	def __init__(self):
		super(testing_motionsSM, self).__init__()
		self.name = 'testing_motions'

		# parameters of this behavior
		self.add_parameter('Kp', 2000)
		self.add_parameter('Kr', 30)
		self.add_parameter('max_vel', 1)
		self.add_parameter('max_acl', 3)
		self.add_parameter('robot', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:968 y:312, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.j_init_pose = "emo/j_init_pose"
		_state_machine.userdata.t2_data = []
		_state_machine.userdata.offset = [0,0,0]
		_state_machine.userdata.rotation = [0,0,0]
		_state_machine.userdata.test_db_pose = "test/some_tf_pose"
		_state_machine.userdata.robot_1_namespace = "panda_1"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:103 y:34
			OperatableStateMachine.add('error_recovery',
										reconcycle_flexbe_states__FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'panda_1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:533 y:427
			OperatableStateMachine.add('CMF_5cm',
										rbs_flexbe_states__RBS_CMoveFor(robot_name='panda_1', dx=[0,0.05,0], t=3, task_space='World'),
										transitions={'continue': 'CMF_-5cm', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 't2_data', 'panda_1': 'panda_1', 'panda_2': 'panda_2', 't2_out': 't2_out'})

			# x:797 y:49
			OperatableStateMachine.add('JMove_init',
										rbs_flexbe_states__RBS_JMove(robot_name="panda_1", q=[1.570, -0.686, 0.0132, -2.5423, -0.229, 1.913, 0.53], t=10, traj='poly'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 't2_data', 'panda_1': 'panda_1', 'panda_2': 'panda_2', 't2_out': 't2_out'})

			# x:562 y:342
			OperatableStateMachine.add('error_rec_2',
										rbs_flexbe_states__RBS_FrankaErrorRecoveryActionProxy(robot_name="panda_1"),
										transitions={'continue': 'CMF_5cm', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:927 y:78
			OperatableStateMachine.add('init_cmove',
										rbs_flexbe_states__RBS_CMove(robot_name="panda_1", x=[0.5, 0, 0.3, 90, 0 ,180], t=10, local_offset=0, global_pos_offset=0, limit_rotations=False),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 't2_data', 'panda_1': 'panda_1', 'panda_2': 'panda_2', 't2_out': 't2_out'})

			# x:406 y:42
			OperatableStateMachine.add('panda_1',
										rbs_flexbe_states__Instantiate_robotblockset(robot_namespace="panda_1"),
										transitions={'continue': 'error_rec_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'panda_1': 'panda_1', 'panda_2': 'panda_2'})

			# x:576 y:509
			OperatableStateMachine.add('CMF_-5cm',
										rbs_flexbe_states__RBS_CMoveFor(robot_name='panda_1', dx=[0,-0.05,0], t=3, task_space='World'),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'t2_data': 't2_data', 'panda_1': 'panda_1', 'panda_2': 'panda_2', 't2_out': 't2_out'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
