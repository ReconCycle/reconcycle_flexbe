#!/usr/bin/python

from flexbe_core import EventState, Logger
import numpy as np
from robotblockset_python.panda_ros import panda_ros

class InitPandaState(EventState):
    '''
    Initializes the Panda robot.
    -- robot_name   string Name of the robot.
    <= continue     Continue with execution.
    <= failed       Failed to initialize the robot.
    '''
    
    def __init__(self, robot_name):
        super(InitPandaState, self).__init__(outcomes = ['continue', 'failed'], 
                                             input_keys = ['robots'],
                                             output_keys = ['robots'])
        self.robot_name = robot_name
        self.done = False
        self.out = 'failed'
    def on_enter(self, userdata):
        if not hasattr(userdata, 'robots') or userdata.robots==None:
            userdata.robots = dict()
        try:
            robot = panda_ros(self.robot_name, init_node=False, init_frankadesk_gripper_TCP=True, 
                            start_controller='position_joint_trajectory_controller')
            robot._verbose = -1 # mute messages!
            robot.Stop_controller()
            robot.SetJointImpedanceFranka(np.array([14000]*7), restart=False)
            robot.SetCollisionBehavior(F=70, T=20, tq=25, restart=False)
            robot.Start_controller()
            robot.error_recovery()
            Logger.logwarn("finished robot init!")
            robot.GetState()                
            userdata.robots[self.robot_name] = robot
            self.out = 'continue'
            self.done = True
        except Exception as e:
            Logger.logwarn("{}".format(e))
            self.out = 'failed'
            self.done = True

    def execute(self, userdata):
        #Logger.logwarn("execute {}".format(self.done))
        #return 'continue'
        if self.done == True:
            return self.out

    def on_exit(self, userdata):
        pass

if __name__ == '__main__':
    import rospy
    class Userdata:
        def init(self):
            self.robots = None

    userdata = Userdata()
    rospy.init_node('panda_test', anonymous = False)
    st = InitPandaState(robot_name = 'panda_1')
    st.on_enter(userdata)
    print(userdata.robots['panda_1'].GetState())
