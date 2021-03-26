#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from myflexgit_flexbe_states.ExecuteJointDMPfromMongoDB import ExeJointDMP
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jan 18 2021
@author: Rok Pahic
'''
class TestDMPexecutionSM(Behavior):
	'''
	Testing DMP execution on robot 2
	'''


	def __init__(self):
		super(TestDMPexecutionSM, self).__init__()
		self.name = 'TestDMPexecution'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:287 y:595, x:55 y:600
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['trj_name'])
		_state_machine.userdata.trj_name = 'breaking_object_n1'
		_state_machine.userdata.True1 = True
		_state_machine.userdata.False1 = False
		_state_machine.userdata.trj_umik = 'trj_umik2'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:267 y:92
			OperatableStateMachine.add('Execute test DMP',
										ExeJointDMP(time_scale=1, motion_timestep=0.01, robot_namespace='panda_2', max_vel=0.3, max_acl=0.3),
										transitions={'continue': 'Retreat', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'entry_name': 'trj_name', 'success': 'success'})

			# x:507 y:82
			OperatableStateMachine.add('Retreat',
										ExeJointDMP(time_scale=1, motion_timestep=0.01, robot_namespace='panda_2', max_vel=0.5, max_acl=0.5),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'entry_name': 'trj_umik', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
