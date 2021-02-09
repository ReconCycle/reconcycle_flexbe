#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from myflexgit_flexbe_states.MoveSoftHand import MoveSoftHand
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jan 18 2021
@author: Rok Pahic
'''
class Grip_testSM(Behavior):
	'''
	Testing Pick and drop in Reconcycle
	'''


	def __init__(self):
		super(Grip_testSM, self).__init__()
		self.name = 'Grip_test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:797 y:131, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goal_joint_pos = [-0.19,-0.18,0.55,-1.71,0.12,1.76,0.79]
		_state_machine.userdata.entry_name = 'test flexbe'
		_state_machine.userdata.goal_joint_pos2 = [-0.74,0.14,0.36,-1.53,-0.07,1.88,0.25]
		_state_machine.userdata.hand_grab_positon = [0.5]
		_state_machine.userdata.hand_release_positon = [0.1]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:163 y:156
			OperatableStateMachine.add('Releasw',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'Grab object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'hand_release_positon', 'success': 'success'})

			# x:577 y:293
			OperatableStateMachine.add('Grab object',
										MoveSoftHand(motion_duration=3, motion_timestep=0.1),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'goal_hand_pos': 'hand_grab_positon', 'success': 'success'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
