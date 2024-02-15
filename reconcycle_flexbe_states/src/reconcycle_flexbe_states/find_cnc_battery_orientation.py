#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from robotblockset_python.transformations import q2rpy
import numpy as np

class FindCNCBatteryRotation(EventState):
	'''
	'''

	def __init__(self, robot_name, vision_utils_name):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(FindCNCBatteryRotation, self).__init__(outcomes = ['continue', 'failed'],
											   input_keys = ['robots', 'vision_utils', 'disassembly_object'],
											   output_keys = ['disassembly_object'])
		

	
		self.robot_name = robot_name
		self.vision_utils_name = vision_utils_name

		self.out = 'failed'

	def on_enter(self, userdata):
		
		r = userdata.robots[self.robot_name]
		vision_utils = userdata.vision_utils[self.vision_utils_name]

		SUCCESS = 0
		try:
			battery_detections = vision_utils.get_detections_with_timeout(desired_class = 'battery_covered', timeout = 4.0)
		except Exception as e:
			Logger.logerr(f'{e}')

		if (battery_detections is not None) and (len(battery_detections) > 0):
			battery_detection = battery_detections[0]
			SUCCESS = 1
			self.out = 'continue'

		# MAX_N_RETRIES = 2
		# n_retry = 0
		# SUCCESS = 0
		# while (SUCCESS==0) and (n_retry<MAX_N_RETRIES):
		# 	try:
		# 		vision_utils.update_detections(timeout=2)
		# 		battery_det = vision_utils.get_particular_class_from_detections(desired_class = 'battery_covered')[0]
		# 		SUCCESS = 1
		# 		Logger.loginfo("Found battery")
		# 	except Exception as e:
		# 		n_retry+=1
		# 		Logger.logerr("Exception", e)
		# 		Logger.logerr("Failed finding battery")
		
		z_rot_deg = 0 # default value if failed at finding
		if SUCCESS:
			rot = battery_detection.tf_px.rotation
			bat_q = [rot.w, rot.x, rot.y, rot.z]

			camera_to_bat_rpy = q2rpy(bat_q)* 180/np.pi
			z_rot_deg = -camera_to_bat_rpy[0] # minus because gripper Z is facing downwards. 
			Logger.loginfo(f'Detected battery rotation {z_rot_deg}')
		else:
			Logger.logerr("Did not detect battery in hekatron. check camera and vision_pipeline. Returning battery rotation 0deg")
		
		userdata.disassembly_object.battery_rotation = z_rot_deg

		self.out = 'continue'

	def execute(self, userdata):

		return self.out
	
	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		self._start_time = rospy.Time.now()


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
