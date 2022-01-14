#!/usr/bin/python3

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
import tf2_geometry_msgs
import tf2_ros

#for deblocking parallel execution in flexbe
import threading

class ReadFromMongoPOSE(EventState):

    '''
    Implements a state that reads TF data from MongoDB
    [...]

    ># entry_name string/int        The name under which the frame is stored in MongoDB

    #< tf_data TransformStamped()   The TF data read from mongoDB (saved as entry_name)

    <= continue                   Written successfully
    <= failed                     Failed
    '''
    
    def __init__(self, robot_ns):
        super(ReadFromMongoPOSE, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['entry_name'], output_keys = ['pose'])
        self.robot_ns = robot_ns
        self.reachable = True

        self.msg_store = MessageStoreProxy()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def read_from_mongodb(self, entry_name):
        data = self.msg_store.query_named(str(entry_name), TransformStamped._type)
        return data

    def transform_to_pose(self, tf):
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation.x = tf.transform.rotation.x
        pose.pose.orientation.y = tf.transform.rotation.y
        pose.pose.orientation.z = tf.transform.rotation.z
        pose.pose.orientation.w = tf.transform.rotation.w

        return pose

    def execute(self, userdata):
        pass
            
    def on_enter(self, userdata):
        userdata.pose = Pose()

        self.entry_name = userdata.entry_name
        try:
            tf_data = self.read_from_mongodb(self.entry_name)[0]

        except rospy.ServiceException as e:
            Logger.loginfo("MongoDB is not reachable...")
            self.reachable = False
            return 'failed'

        tf_source_frame = tf_data.header.frame_id
        tf_target_frame = tf_data.child_frame_id
        tf_robot_frame = self.robot_ns + '/' + self.robot_ns + '_link0'

        if tf_source_frame != tf_robot_frame:
            Logger.loginfo("Looking up transform from {} to {}.".format(
                tf_robot_frame, tf_source_frame))
            tf_robot_to_source_frame = self.buffer.lookup_transform(
                    tf_robot_frame, tf_source_frame, rospy.Time(0), rospy.Duration(2))

            # Logger.loginfo("TF:\n{}".format(tf_robot_to_source_frame))
            tf_r2s_pose = self.transform_to_pose(tf_robot_to_source_frame)
            tf_transformed = tf2_geometry_msgs.do_transform_pose(tf_r2s_pose, tf_data)
            userdata.pose = tf_transformed
        else:
            userdata.pose = self.transform_to_pose(tf_data)
        Logger.loginfo("Reading _id: {} from mongoDB: ... \n {}".format(userdata.entry_name, userdata.pose))      
        return 'continue'  
    
    
    def on_exit(self, userdata):
        if self.reachable:
            Logger.loginfo("Finished reading from MongoDB.")
        else:
            Logger.loginfo("Could not read from MongoDB!")
        
if __name__ == '__main__':
     print("Testing standalone")
     rospy.init_node('test_node')
     class UserData:
         entry_name = "tf_testing_2"
     userdata = UserData
     test_state = ReadFromMongoPOSE(robot_ns='panda_2')
     test_state.on_enter(userdata)
