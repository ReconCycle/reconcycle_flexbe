#!/usr/bin/env python

import rospy
import mongodb_store_msgs.srv as dc_srv
from mongodb_store.message_store import MessageStoreProxy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import Pose
import StringIO
import tf

class WriteToMongoPOSE(EventState):

    '''
    Implements a state that writes data to MongoDB
    [...]       

    ># position     array           Data to write into MongoDB postion = [x,y,z]
    ># entry_name   string/int      The name under which the name is written into MongoDB
    ># orientation  array           [xrot,yrot,zrot]

    <= continue                     Written successfully
    <= failed                       Failed
    '''
    
    #-- calculation  function        The function that performs [...]

    def __init__(self):
        rospy.loginfo('__init__ callback happened.')        
        super(WriteToMongoPOSE, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['position','orientation','entry_name'])
        self.reachable = True

        self.msg_store = MessageStoreProxy()

    def execute(self, userdata):
        #---------------------------------------------------------------------------------------
        # userdata.entry_data for JointState format
        position = userdata.position
        rotation = userdata.rotation
        #---------------------------------------------------------------------------------------
        rotation = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])

        # Write to MongoDB        
        Logger.loginfo("Writing to mongoDB _id: {}...".format(userdata.entry_name))

        pos = Pose(position=postion, orientation=rotation)
        
        try: 
            present_data = self.msg_store.query_named(str(userdata.entry_name), Pose._type)
            Logger.loginfo("Data in DB: {}".format(present_data)) 
            if present_data[0] != None:
                Logger.loginfo("Id {} already exists in DB...Updating data.".format(userdata.entry_name))
                client= self.msg_store.update_named(str(userdata.entry_name), pos)
            else:
                Logger.loginfo("Writing to DB: id {}, positon {}, rotation {}.".format(userdata.entry_name, userdata.position, rotation))
                client = self.msg_store.insert_named(str(userdata.entry_name), pos)
                   
            Logger.loginfo("Written successfully!")

        except rospy.ServiceException as e:
            Logger.loginfo("MongoDB is not reachable...")
            self.reachable = False
    
        return 'continue'
             
    def on_enter(self, userdata):
        Logger.loginfo('Starting write to MongoDB...')

    def on_exit(self, userdata):
        if self.reachable:
            Logger.loginfo("Finished writting to MongoDB!")
        else:
            Logger.loginfo("Could not write to MongoDB!")
