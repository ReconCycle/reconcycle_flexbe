#!/usr/bin/python3

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose

#for deblocking parallel execution in flexbe
import threading

class ReadFromMongoPOSE(EventState):

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
        super(ReadFromMongoPOSE, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['entry_name'], output_keys = ['joints_data'])
        self.reachable = True

        self.msg_store = MessageStoreProxy()


    def read_from_mongodb(self):   

        self.read_data= self.msg_store.query_named(str(self.entry_name), Pose._type)

    def execute(self, userdata):
        pos = Pose()

        self.entry_name = userdata.entry_name
        try:
            #data_from_db = self.msg_store.query_named(str(userdata.entry_name), JointState._type)
            #threading.Thread(target=self.read_from_mongodb).start()
            self.read_from_mongodb()

            data_from_db = self.read_data
            position_data = list(data_from_db.position) + list(data_from_db.orientation)
            Logger.loginfo("Position data read from DB: \n {}".format(position_data))      

        except rospy.ServiceException as e:
            Logger.loginfo("MongoDB is not reachable...")
            self.reachable = False
            return 'failed'

        userdata.joints_data = position_data
        Logger.loginfo("Reading _id: {} from mongoDB: ... \n {}".format(userdata.entry_name, userdata.joints_data))      
        return 'continue'  
            
    def on_enter(self, userdata):
        Logger.loginfo("Starting read from MongoDB...")

    def on_exit(self, userdata):
        if self.reachable:
            Logger.loginfo("Finished reading from MongoDB.")
        else:
            Logger.loginfo("Could not read from MongoDB!")
        
