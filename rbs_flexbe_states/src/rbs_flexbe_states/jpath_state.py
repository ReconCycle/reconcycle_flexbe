#!/usr/bin/python

from flexbe_core import EventState, Logger

class JPathState(EventState):

    def __init__(self, robot_name, path, t):
        super(JPathState, self).__init__(
            outcomes = ['continue', 'failed'],
            input_keys = ['robots']
            )

        self.robot_name = robot_name
        self.jpath = path
        self.t = t
        
    def on_enter(self, userdata):
        
        robot = userdata.robots[self.robot_name]
        # JPath requires first q pose to be the actual robot pose.
        path_with_initial_pose_added = []
        
        robot.GetState()
        path_with_initial_pose_added.append(robot._actual_int.q)
        for path_point in self.jpath:
            path_with_initial_pose_added.append(path_point)

        userdata.robots[self.robot_name].JPath(
            path_with_initial_pose_added,self.t)

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass

if __name__ == '__main__':
    import rospy
    rospy.init_node('panda_test', anonymous = False)

    from flexbe_core.core.user_data import UserData
    from init_reconcycle_panda_state import InitReconcyclePandaState

    ud = UserData(input_keys=['robots'])
    ud.robots= None
    st = InitReconcyclePandaState(robot_name = 'panda_1',tool_name=None)
    st.on_enter(ud)
    st.execute(ud)

    path=[rospy.get_param('/pose_db/panda_1/via_cnc_vision/joints'), rospy.get_param('/pose_db/panda_1/above_cnc/joints')]

    st2 = JPathState(robot_name = 'panda_1', path = path, t = 4)

    st2.on_enter(ud)
    st2.execute(ud)

