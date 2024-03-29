#!/usr/bin/python3

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose

#for deblocking parallel execution in flexbe
import threading

class ReadFromMongoPOSEMult(EventState):

    '''
    Implements a state that writes data to MongoDB
    [...]

    ># entry_name string/int      The name under which the name is written into MongoDB

    #< joints_data Pose()         The data read from mongoDB from specific _id (entry_name)

    <= continue                   Written successfully
    <= failed                     Failed
    '''
    

    #-- calculation  function        The function that performs [...]
    def __init__(self):      
        super(ReadFromMongoPOSEMult, self).__init__(outcomes = ['continue', 'failed'], 
                                                    input_keys = ['entry_name_array'], output_keys = ['joints_data'])
        self.reachable = True

        self.msg_store = MessageStoreProxy()


    def read_from_mongodb(self):   


        self.read_data = self.msg_store.query_named(str(self.entry_name), Pose._type)

    def execute(self, userdata):
        pass

            
    def on_enter(self, userdata):
        pos = Pose()

        position_data_array = []

        for entry in userdata.entry_name_array:
            
            self.entry_name = entry
            try:
                #data_from_db = self.msg_store.query_named(str(userdata.entry_name_array), JointState._type)
                #threading.Thread(target=self.read_from_mongodb).start()
                self.read_from_mongodb()

                data_from_db = self.read_data
                position_data = data_from_db
                Logger.loginfo("Position data read from DB: \n {}".format(position_data))      

            except rospy.ServiceException as e:
                Logger.loginfo("MongoDB is not reachable...")
                self.reachable = False
                return 'failed'

            position_data_array.append(position_data)
        
        userdata.joints_data = position_data_array
        Logger.loginfo("Reading _id: {} from mongoDB: ... \n {}".format(userdata.entry_name_array, userdata.joints_data))      
        return 'continue'  
    
    
    def on_exit(self, userdata):
        if self.reachable:
            Logger.loginfo("Finished reading from MongoDB.")
        else:
            Logger.loginfo("Could not read from MongoDB!")
        
if __name__ == '__main__':
    print("Testing standalone")
    rospy.init_node('test_node')

    class UserData():
        entry_name_array = ["test_pose_1", "test_pose_2"]
    
    udata = UserData()
    test_state = ReadFromMongoPOSEMult()
    test_state.on_enter(udata)
