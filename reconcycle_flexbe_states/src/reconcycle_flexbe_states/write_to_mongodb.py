#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
#import mongodb_store.util as dc_util 
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from sensor_msgs.msg import JointState
try:
    from StringIO import StringIO ## for Python 2
except ImportError:
    from io import StringIO ## for Python 3


class WriteToMongo(EventState):

    '''
    Implements a state that writes data to MongoDB
    [...]       

    ># entry_data  JointState()  Data to write into MongoDB (sensor_msgs.msg -> JointState() fromat!)
    ># entry_name  string/int    The name under which the name is written into MongoDB

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #-- calculation  function        The function that performs [...]

    def __init__(self):
        rospy.loginfo('__init__ callback happened.')        
        super(WriteToMongo, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['entry_data','entry_name'])
        self.reachable = True

        self.msg_store = MessageStoreProxy()

    def execute(self, userdata):
        #---------------------------------------------------------------------------------------
        # userdata.entry_data for JointState format
        entry_data = userdata.entry_data
        #---------------------------------------------------------------------------------------

        # Write to MongoDB        
        Logger.loginfo("Writing to mongoDB _id: {}...".format(userdata.entry_name))

        pos = JointState(position=entry_data)
        
        try: 
            present_data = self.msg_store.query_named(str(userdata.entry_name), JointState._type)
            Logger.loginfo("Data in DB: {}".format(present_data)) 
            if present_data[0] != None:
                Logger.loginfo("Id {} already exists in DB...Updating data.".format(userdata.entry_name))
                client= self.msg_store.update_named(str(userdata.entry_name), pos)
            else:
                Logger.loginfo("Writing to DB: id {}, data {}.".format(userdata.entry_name, userdata.entry_data))
                client = self.msg_store.insert_named(str(userdata.entry_name), pos)
                   
            Logger.loginfo("Written successfully!")

        except rospy.ServiceException as e:
            Logger.loginfo("MongoDB is not reachable...")
            self.reachable = False
    
        return 'continue'
             
    def on_enter(self, userdata):
        0#Logger.loginfo('Starting write to MongoDB...')

    def on_exit(self, userdata):
        if self.reachable:
            Logger.loginfo("Finished writting to MongoDB!")
        else:
            Logger.loginfo("Could not write to MongoDB!")
