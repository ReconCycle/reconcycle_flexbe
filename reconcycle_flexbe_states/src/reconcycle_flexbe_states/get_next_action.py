#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from franka_msgs.srv import *

from context_action_framework.srv._NextAction import NextAction, NextActionRequest, NextActionResponse

class GetNextAction(EventState):

    '''
    FlexBe state that calls the Action predictor to get the next action
    
    Parameters as follow

    
    <= continue
    <= failed

    '''
    
    def __init__(self):
        super(GetNextAction, self).__init__(outcomes = ['continue', 'failed'])
        
        # Input params 
        
        self.topic = "/action_predictor/get_next_action"
        self.client_proxy = ProxyServiceCaller({self.topic: NextAction}, wait_duration = 1)

    def on_enter(self, userdata):
        
        Logger.loginfo("Started service GetNextAction...")
        
        request = NextActionRequest()
        
        # Checks
        assert userdata.success_prev in ['True', 'False', 'true', 'false', True, False]
              
        request.success_prev = userdata.success_prev
        request.uuid_prev = userdata.uuid_prev
        request.action_type_prev = userdata.action_type_prev
        request.action_details = userdata.action_details_prev
        
        try:
            if self.client_proxy.is_available(self.topic):
                response = self.client_proxy.call(self.topic, request)
                
                # Handle action_predictor output back to userdata
                userdata.success_prev = response.success
                userdata.uuid_prev = response.uuid
                userdata.action_type_prev = response.action_type
                #userdata.action_details_prev = response.action_details
                userdata.action_block_prev = response.action_block
                
                rospy.loginfo("Success: {}".format(response.success))
                rospy.loginfo("UUID: {}".format(response.uuid))
                rospy.loginfo("Next action: {}".format(response.action_type))
                rospy.loginfo("Next action: {}".format(response.action_block))
                                
            else:
                rospy.loginfo("Service is not available!")
                return 'failed'
            
        #except (CommandException, NetworkException) as e:
        except Exception as e:
            rospy.loginfo(e)
            return 'failed'        

    def execute(self, userdata):
        return 'continue'
        

    def on_exit(self, userdata):
        Logger.loginfo("Finished service client!")
        return 'continue'

    def on_start(self):
        pass

    def on_stop(self):
        pass
