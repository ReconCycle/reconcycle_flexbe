#!/usr/bin/env python


#ros stuf
import rospy


#flexbe stuff
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

#mongo stuff
from mongodb_store.message_store import MessageStoreProxy

import time

# Action messages
from robot_module_msgs.msg import JointDMPAction,JointDMPGoal
from robot_module_msgs.msg import JointTrapVelAction,JointTrapVelGoal
#from robot_module_msgs.msg import JointMinJerkAction,JointMinJerkGoal

from robot_module_msgs.msg import JointSpaceDMP

from sensor_msgs.msg import JointState
class ExeJointDMP(EventState):

    '''
    Calls qb hand server
    [...]
    
    -- time scale      float	Scale Tau of DMP trajectory
    -- motion_timestep  float		Timestamp data  
    ># goal_hand_pos   string []	Position data input [float] of 1 joint
    #< success     bool 	minjerk action server reply userdata output
    <= continue                 Written successfully
    <= failed                  	Failed
    '''

    def __init__(self, time_scale, motion_timestep, robot_namespace='', max_vel=0.5, max_acl=0.5):
        super( ExeJointDMP, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['entry_name'], output_keys = ['success'])
        Logger.loginfo("INIT...DMP execution")

        self._robot_namespace = robot_namespace
        # save max values for trap vel move
        self._max_acl=max_acl
        self._max_vel=max_vel

        self._move_joint_topic = self._robot_namespace + '/joint_impedance_controller/move_joint_trap'
        self._execute_DMP_topic = self._robot_namespace + '/joint_DMP_action_server'
        self._execute_DMP_topic = self._robot_namespace + '/joint_impedance_controller/move_joint_DMP'
        #self.DMP_goal='/joint_impedance_controller/move_joint_DMP/goal'


        # Client for reading from mongoDB

        #self._client_mongodb  = MessageStoreProxy()
        
        # Action client for moving on start position 

        # Action client for executing DMP

        self._client = ProxyActionClient({ self._move_joint_topic: JointTrapVelAction,self._execute_DMP_topic: JointDMPAction})


     


        self._time_scale=time_scale
        self._timestep = motion_timestep
        self._duration = 6
        
    def on_enter(self, userdata):

        Logger.loginfo("Reading from DMP from MongoDB base: \n {}".format('DMP name'+userdata.entry_name))



          #Read from mongo db 
     
        Logger.loginfo("Reading _id: {} DMP from mongoDB: ... \n ".format(userdata.entry_name))  
 

        try:
   
            # data_from_db = self._client_mongodb.query_named(userdata.entry_name, JointSpaceDMP._type)

	    import yaml
            with open("/ros_ws/src/reconcycle_states/trj2","r") as f:
		data=yaml.load(f)

            from robot_module_msgs.msg  import Float32Array
            g=JointDMPGoal()
            g.DMP.tau=data['tau']
            g.DMP.c=data['c']
            g.DMP.goal=data['goal']
            g.DMP.a_z=data['a_z']
            g.DMP.d_t=data['d_t']
            g.DMP.N=data['N']
            g.DMP.b_z=data['b_z']
            for i in range(0,7):
		#g.DMP.w=data['w']
                g.DMP.w.append(Float32Array())
                g.DMP.w[i].data=


	        g.DMP.y0=data['y0']
                g.DMP.sigma=data['sigma']
            g.DMP.a_x=data['a_x']


        except rospy.ServiceException as e:
    
            Logger.loginfo("MongoDB is not reachable...")
            self.reachable = False
            return 'failed'

        #Move robot on DMP starting postion y0
        Logger.loginfo("Move robot on DMP starting postion y0...")

        joint = JointState()
        joint.position = DMP.y0.position
        
        start_goal = JointTrapVelGoal([joint], self._max_vel, self._max_acl)
        #start_goal = JointMinJerkGoal(DMP.y0.position, self._duration, self._timestep)
        

        try:
            Logger.loginfo("Start position sent: {}".format(DMP.y0.position))


            self._client.send_goal(self._move_joint_topic,start_goal)
            timeout = time.time()
            while self._client.get_result(self._move_joint_topic) == None:

                feedback = self._client.get_feedback(self._move_joint_topic)
                Logger.loginfo("{}".format(feedback))
                time.sleep(0.5)
                # 12 secs timeout
                if time.time()-timeout > 12:
                    return 'failed'#break

        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to send the starting point command:\n{}'.format(str(e)))
            self._error = True
            return 'failed'

        #Execute DMP 
        Logger.loginfo("Execute DMP ...")

        
        dmp_goal = JointDMPGoal(DMP)
        

        try:
            #Logger.loginfo("DMP sent: {}".format(DMP))


            self._client.send_goal(self._execute_DMP_topic,dmp_goal)
            timeout = time.time()
            while self._client.get_result(self._execute_DMP_topic) == None:
                time.sleep(0.5)
                feedback = self._client.get_feedback(self._execute_DMP_topic)
                
                
                Logger.loginfo("{}".format(str(feedback.joints.position)))

                
                # 12 secs timeout
                if time.time()-timeout > 12:
                    return 'failed'


        except Exception as e:
            # Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
            Logger.loginfo('Failed to send DMP:\n{}'.format(str(e)))
            self._error = True
            return 'failed' 
            
      

    def execute(self, userdata):

      
        
        return 'continue'



            
   

    def on_exit(self, userdata):

        Logger.loginfo('Finished sending goal to hand.')
        if not self._client.get_result(self._execute_DMP_topic):
            self._client.cancel(self._execute_DMP_topic)
            Logger.loginfo('Cancelled active action goal. No reply data.')
        Logger.loginfo('Finished sending goal to hand.')
        return 'continue'





if __name__ == '__main__':

    print("Testing standalone")

    class userdata():

        def __init__(self,name):

            self.entry_name=name

    usertest=userdata("trj1")
    rospy.init_node('test_node')
    test_state=ExeJointDMP(1,0.1,robot_namespace='')

    #test_state.on_enter(usertest)
    test_state.execute(usertest)
    test_state.on_exit(usertest)
