#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from reconcycle_flexbe_states.ExecuteJointDMPfromMongoDB import ExeJointDMP
from reconcycle_flexbe_states.call_joint_trap_vel_action_server import CallJointTrap
from reconcycle_flexbe_states.read_from_mongodb import ReadFromMongo
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jan 18 2021
@author: Rok Pahic
'''
class BreakingobjectSM(Behavior):
	'''
	Testing DMP execution on robot 2
	'''


	def __init__(self):
		super(BreakingobjectSM, self).__init__()
		self.name = 'Breaking object'

		# parameters of this behavior
		self.add_parameter('max_vel', 0.3)
		self.add_parameter('max_acl', 0.3)
		self.add_parameter('namespace', 'panda_2')

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
		_state_machine.userdata.safe_position_name = 'panda2_beffore_clamp'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_location_2_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_location_2_0:
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


		# x:30 y:365, x:130 y:365
		_sm_move_to_safe_location_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['position_name'])

		with _sm_move_to_safe_location_1:
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



		with _state_machine:
			# x:62 y:55
			OperatableStateMachine.add('Move to safe location',
										_sm_move_to_safe_location_1,
										transitions={'finished': 'Execute test DMP', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'safe_position_name'})

			# x:722 y:227
			OperatableStateMachine.add('Move to safe location_2',
										_sm_move_to_safe_location_2_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'position_name': 'safe_position_name'})

			# x:507 y:82
			OperatableStateMachine.add('Retreat',
										ExeJointDMP(time_scale=1, motion_timestep=0.01, robot_namespace='panda_2', max_vel=0.5, max_acl=0.5),
										transitions={'continue': 'Move to safe location_2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'entry_name': 'trj_umik', 'success': 'success'})

			# x:267 y:92
			OperatableStateMachine.add('Execute test DMP',
										ExeJointDMP(time_scale=1, motion_timestep=0.01, robot_namespace='panda_2', max_vel=0.3, max_acl=0.3),
										transitions={'continue': 'Retreat', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'entry_name': 'trj_name', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
