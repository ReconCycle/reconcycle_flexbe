#!/usr/bin/env python
import rostopic
from flexbe_core import EventState, Logger
from cv_bridge import CvBridge, CvBridgeError
from flexbe_core.proxy import ProxySubscriberCached
import matplotlib.pyplot as plt
import threading
import cv2

class ShowImage(EventState):
    '''
    Gets the latest message on the given topic and stores it to userdata.

    -- topic 		string		The topic on which should be listened.
    -- blocking 	bool 		Blocks until a message is received.
    -- clear 		bool 		Drops last message on this topic on enter
                                in order to only handle message received since this state is active.

    #> message		object		Latest message on the given topic of the respective type.

    <= received 				Message has been received and stored in userdata or state is not blocking.
    <= unavailable 				The topic is not available when this state becomes actives.
    '''

    def __init__(self, topic, blocking=True, clear=False):
        super(ShowImage, self).__init__(outcomes=['received', 'unavailable'],
                                              output_keys=['message'])
        self._topic = topic
        self._blocking = blocking
        self._clear = clear
        self._connected = False
        self._bridge = CvBridge()
        self._outcome = None

        #self.visualization_thread = threading.Thread(target=self.visualizationLoop)
        #self.visualization_thread.daemon = True
        
        if not self._connect():
            Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._topic, self.name))

    def execute(self, userdata):
        if not self._connected:
            userdata.message = None
            self._outcome = 'unavailable'

        if self._sub.has_msg(self._topic) or not self._blocking:
            userdata.message = self._sub.get_last_msg(self._topic)
            self._sub.remove_last_msg(self._topic)
            
            self.cv_image = self._bridge.imgmsg_to_cv2(userdata.message, "passthrough")       
            self._outcome = 'received'
        
        return self._outcome

    def on_enter(self, userdata):
        if not self._connected:
            if self._connect():
                Logger.loginfo('Successfully subscribed to previously unavailable topic %s' % self._topic)
            else:
                Logger.logwarn('Topic %s still not available, giving up.' % self._topic)

        if self._connected and self._clear and self._sub.has_msg(self._topic):
            self._sub.remove_last_msg(self._topic)

    def _connect(self):
        msg_type, msg_topic, _ = rostopic.get_topic_class(self._topic)
        if msg_topic == self._topic:
            self._sub = ProxySubscriberCached({self._topic: msg_type})
            self._connected = True
            return True
        return False
    
    def on_exit(self, userdata):
        
        #print('Creating cv2 window')
        #window_name = 'Segmentation results'
        #cv2.namedWindow(window_name)
        #cv2.imshow(window_name, self.cv_image)
    
        #plt.imshow(self.cv_image) 
        #plt.show(block=False)
        #plt.show()
        
        cv2.imwrite('/ros_ws/src/reconcycle_states/yolact-res.jpg' , self.cv_image)
        return self._outcome

