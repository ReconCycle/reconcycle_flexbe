
import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import JointState

'''
Created on 06.03.2016
@author: Philipp Schillinger
'''

class GetJointValuesState(EventState):
	'''
	Retrieves current values of specified joints.
	-- joints		string[]	List of desired joint names.
	#> joint_state JointState() List of current joint values.
	<= retrieved 				Joint values are available.
	'''

	def __init__(self, joints):
		'''
		Constructor
		'''
		super(GetJointValuesState, self).__init__(outcomes=['retrieved'],
												output_keys=['joint_state'])
		
		self._topic = '/joint_states'
		self._sub = ProxySubscriberCached({self._topic: JointState})

		self._joints = joints
		self._joint_state = JointState
			
		
	def execute(self, userdata):

		while self._sub.has_buffered(self._topic):
			msg = self._sub.get_from_buffer(self._topic)
			self._joint_state = msg
			
	def on_enter(self, userdata):
		self._sub.enable_buffer(self._topic)
		self._joint_state = JointState

	def on_exit(self, userdata):
		self._sub.disable_buffer(self._topic)