#!/usr/bin/env python



import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import robot_module_msgs.msg
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import time




class MoveSoftClaw(EventState):

    '''
    Calls qb softclaw server
    [...]
    
    -- motion_duration  float		Speed data
      
    ># goal_claw_pos   string []	Position data input [float] of 2 joints
    #< success     bool 	minjerk action server reply userdata output
    <= continue                 Written successfully
    <= failed                  	Failed
    '''

    def __init__(self, motion_duration):
        super( MoveSoftClaw, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['goal_claw_pos'], output_keys = ['success'])
        Logger.loginfo("Soft claw initialization...")

        # actionlib client @move 
        #FOR NOW #self._client = actionlib.SimpleActionClient('joint_min_jerk_action_server', robot_module_msgs.msg.JointMinJerkAction)

        self._topic = '/qbsoftclaw/control/qbsoftclaw_position_and_preset_trajectory_controller/follow_joint_trajectory'
        self._client = ProxyActionClient({self._topic: FollowJointTrajectoryAction})

        #self._client = ProxyActionClient({self._topic: robot_module_msgs.msg.JointMinJerkAction}) # pass required clients as dict (topic: type)
        # JointMinJerkAction

        # Input parameters of robot_module_msgs.msg
        self._duration = rospy.Duration(motion_duration)
        

    def execute(self, userdata):
        return 'continue'
            
    def on_enter(self, userdata):

        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = ['qbsoftclaw_shaft_joint', 'qbsoftclaw_stiffness_preset_virtual_joint']

        point = JointTrajectoryPoint()
        point.positions = userdata.goal_claw_pos
        point.velocities = userdata.goal_claw_vel
        point.accelerations = userdata.goal_claw_acc
        point.effort = userdata.goal_claw_effort
        point.time_from_start = self._duration

        goal.trajectory.points = [point]

        Logger.loginfo("Starting sending goal...")

        try:
            Logger.loginfo("Goal sent: {}".format(str(userdata.goal_claw_pos)))
            self._client.send_goal(self._topic, goal)
            while self._client.get_result(self._topic) == None:
                # Logger.loginfo("{}".format(robot_module_msgs.msg.JointMinJerkFeedback()))
                time.sleep(0.2)

        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to send the goal command:\n{}'.format(str(e)))
            self._error = True
        
        result = []
        try:
            result = self._client.get_result(self._topic)
            Logger.loginfo("Action Server reply: \n {}".format(str(result)))
        except Exception as e:
            Logger.loginfo("No result or server is not active!")
            return 'failed'
    
    def on_exit(self, userdata):

        if not self._client.get_result(self._topic):
            try: 
                self._client.cancel_goal(self._topic)
                Logger.loginfo('Cancelled active action goal. No reply data.')
            except:
                Logger.loginfo("MoveSoftClaw - FAILED to cancel goal")
        Logger.loginfo('Finished sending goal to hand.')
        return 'continue'
 

if __name__ == '__main__':

    print("Testing standalone")

    class userdata():
        def __init__(self, pos, vel, acc, eff):
            self.goal_claw_pos = [pos, pos]
            self.goal_claw_vel = [vel, vel]
            self.goal_claw_acc = [acc, acc]
            self.goal_claw_effort = [eff, eff]

    usertest = userdata(pos = -0.5, vel = 0.1, acc = 0.1, eff = 0.1)
    rospy.init_node('test_node_qbsoftclaw')
    test_state = MoveSoftClaw(motion_duration= 1)
    test_state.on_enter(usertest)
    test_state.on_exit(usertest)
