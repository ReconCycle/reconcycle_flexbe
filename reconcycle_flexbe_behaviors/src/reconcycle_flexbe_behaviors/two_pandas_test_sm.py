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
from reconcycle_flexbe_states.Move_custom_robot import MoveCustomRobot
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Feb 26 2021
@author: Matej
'''
class TwopandastestSM(Behavior):
	'''
	Simulating concurrency with tow pandas.
	'''


	def __init__(self):
		super(TwopandastestSM, self).__init__()
		self.name = 'Two pandas test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		robot1 = 'panda1'
		robot2 = 'panda2'
		# x:30 y:365, x:32 y:306
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose1 = [1.57,-0.20,0.20,-2.00,0.018,1.83,1]
		_state_machine.userdata.pose2 = [-1.57,-0.20,0.20,-2.00,0.018,1.83,1]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365
		_sm_container_2_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['pose1', 'pose2'], conditions=[
										('failed', [('Move1', 'failed'), ('Move2', 'failed')]),
										('finished', [('Move2', 'continue'), ('Move1', 'continue')])
										])

		with _sm_container_2_0:
			# x:58 y:62
			OperatableStateMachine.add('Move1',
										MoveCustomRobot(robot_selection=robot1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_joint_pos': 'pose2', 'minjerk_out': 'minjerk_out'})

			# x:295 y:73
			OperatableStateMachine.add('Move2',
										MoveCustomRobot(robot_selection=robot2),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_joint_pos': 'pose1', 'minjerk_out': 'minjerk_out'})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_container_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['pose1', 'pose2'], conditions=[
										('finished', [('Moverobot1', 'continue')]),
										('finished', [('Moverobot2', 'continue')]),
										('finished', [('Moverobot2', 'failed')]),
										('failed', [('Moverobot1', 'failed')])
										])

		with _sm_container_1:
			# x:53 y:58
			OperatableStateMachine.add('Moverobot1',
										MoveCustomRobot(robot_selection=robot1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_joint_pos': 'pose1', 'minjerk_out': 'minjerk_out'})

			# x:289 y:61
			OperatableStateMachine.add('Moverobot2',
										MoveCustomRobot(robot_selection=robot2),
										transitions={'continue': 'finished', 'failed': 'finished'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_joint_pos': 'pose2', 'minjerk_out': 'minjerk_out'})



		with _state_machine:
			# x:124 y:51
			OperatableStateMachine.add('Container',
										_sm_container_1,
										transitions={'finished': 'wait', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose1': 'pose1', 'pose2': 'pose2'})

			# x:458 y:51
			OperatableStateMachine.add('Container_2',
										_sm_container_2_0,
										transitions={'finished': 'wait1', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose1': 'pose1', 'pose2': 'pose2'})

			# x:314 y:51
			OperatableStateMachine.add('wait',
										WaitState(wait_time=2),
										transitions={'done': 'Container_2'},
										autonomy={'done': Autonomy.Off})

			# x:482 y:175
			OperatableStateMachine.add('wait1',
										WaitState(wait_time=2),
										transitions={'done': 'Container'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
