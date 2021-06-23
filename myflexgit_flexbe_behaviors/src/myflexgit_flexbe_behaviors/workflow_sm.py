#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.calculation_state import CalculationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jun 17 2021
@author: Rok Pahic
'''
class WorkflowSM(Behavior):
	'''
	For review
	'''


	def __init__(self):
		super(WorkflowSM, self).__init__()
		self.name = 'Workflow'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:614 y:388, x:312 y:260
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.input_value = 4

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:502, x:254 y:494
		_sm_visual_pcb_detection__0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['input_value'])

		with _sm_visual_pcb_detection__0:
			# x:30 y:40
			OperatableStateMachine.add('tests',
										CalculationState(calculation=3),
										transitions={'done': 'tests_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:117
			OperatableStateMachine.add('tests_2',
										CalculationState(calculation=3),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})


		# x:30 y:425, x:142 y:365
		_sm_transport_to_vise_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['input_value'])

		with _sm_transport_to_vise_1:
			# x:30 y:40
			OperatableStateMachine.add('tests',
										CalculationState(calculation=3),
										transitions={'done': 'tests_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:117
			OperatableStateMachine.add('tests_2',
										CalculationState(calculation=3),
										transitions={'done': 'tests_3'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:194
			OperatableStateMachine.add('tests_3',
										CalculationState(calculation=3),
										transitions={'done': 'tests_4'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:271
			OperatableStateMachine.add('tests_4',
										CalculationState(calculation=3),
										transitions={'done': 'tests_5'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:348
			OperatableStateMachine.add('tests_5',
										CalculationState(calculation=3),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})


		# x:30 y:365, x:130 y:365
		_sm_transport_the_battery_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['input_value'])

		with _sm_transport_the_battery_2:
			# x:30 y:40
			OperatableStateMachine.add('tests',
										CalculationState(calculation=3),
										transitions={'done': 'tests_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:117
			OperatableStateMachine.add('tests_2',
										CalculationState(calculation=3),
										transitions={'done': 'tests_3'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:194
			OperatableStateMachine.add('tests_3',
										CalculationState(calculation=3),
										transitions={'done': 'tests_4'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:271
			OperatableStateMachine.add('tests_4',
										CalculationState(calculation=3),
										transitions={'done': 'tests_5'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:411
			OperatableStateMachine.add('tests_5',
										CalculationState(calculation=3),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})


		# x:30 y:365, x:130 y:365
		_sm_transport_the_pcb_to_the_cutting_device_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['input_value'])

		with _sm_transport_the_pcb_to_the_cutting_device_3:
			# x:30 y:40
			OperatableStateMachine.add('tests',
										CalculationState(calculation=3),
										transitions={'done': 'tests_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:117
			OperatableStateMachine.add('tests_2',
										CalculationState(calculation=3),
										transitions={'done': 'tests_3'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:194
			OperatableStateMachine.add('tests_3',
										CalculationState(calculation=3),
										transitions={'done': 'tests_4'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:271
			OperatableStateMachine.add('tests_4',
										CalculationState(calculation=3),
										transitions={'done': 'tests_5'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:411
			OperatableStateMachine.add('tests_5',
										CalculationState(calculation=3),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})


		# x:30 y:380, x:130 y:380
		_sm_pry_out_the_inner_part_(pcb)_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['input_value'])

		with _sm_pry_out_the_inner_part_(pcb)_4:
			# x:30 y:40
			OperatableStateMachine.add('tests',
										CalculationState(calculation=3),
										transitions={'done': 'tests_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:117
			OperatableStateMachine.add('tests_2',
										CalculationState(calculation=3),
										transitions={'done': 'tests_3'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:30 y:194
			OperatableStateMachine.add('tests_3',
										CalculationState(calculation=3),
										transitions={'done': 'tests_5'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:57 y:318
			OperatableStateMachine.add('tests_5',
										CalculationState(calculation=3),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})



		with _state_machine:
			# x:101 y:36
			OperatableStateMachine.add('Transport to vise',
										_sm_transport_to_vise_1,
										transitions={'finished': 'Visual PCB detection ', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_value': 'input_value'})

			# x:556 y:147
			OperatableStateMachine.add('Transport the PCB to the cutting device',
										_sm_transport_the_pcb_to_the_cutting_device_3,
										transitions={'finished': 'Transport the battery', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_value': 'input_value'})

			# x:587 y:260
			OperatableStateMachine.add('Transport the battery',
										_sm_transport_the_battery_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_value': 'input_value'})

			# x:333 y:38
			OperatableStateMachine.add('Visual PCB detection ',
										_sm_visual_pcb_detection__0,
										transitions={'finished': 'Pry out the inner part (PCB)', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_value': 'input_value'})

			# x:579 y:41
			OperatableStateMachine.add('Pry out the inner part (PCB)',
										_sm_pry_out_the_inner_part_(pcb)_4,
										transitions={'finished': 'Transport the PCB to the cutting device', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'input_value': 'input_value'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
