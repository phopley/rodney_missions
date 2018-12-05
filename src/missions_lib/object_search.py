#!/usr/bin/env python
# States for searching for an object

import rospy
from smach import State
from std_msgs.msg import Empty

# MOVE_CAMERA_OD State
class MoveCameraObjectSearch(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['awaiting_movement','complete'],
                       input_keys=['start_in'],
                       output_keys=['start_out'])
        self.__helper_obj = helper_obj                                 
                       
    def execute(self, userdata):
        # Is this a new search?
        if userdata.start_in == True:
            userdata.start_out = False
            scan_complete = False
            # Set the camera position to start position (pan min and tilt min)
            self.__helper_obj.CameraToStartPos()
        else:            
            scan_complete = self.__helper_obj.CameraToNextPos()

        if scan_complete == True:
            greeting = "Mission complete Rodney standing by"
            # Speak
            self.__helper_obj.Speak(greeting, greeting)
            next_outcome = 'complete'
        else:
            next_outcome = 'awaiting_movement'                
        	    
        return next_outcome


# CHOICE_OS State
class ChoiceObjectSearch(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['move_camera','await_user'],
                       input_keys=['object_detected','mission_data'])
        self.__helper_obj = helper_obj
                       
    def execute(self, userdata):
        # Now make the decission on the next state to transit to based on if
        # the object was detected on the last scan
        if userdata.object_detected == True:
            next_outcome = 'await_user'
            # We will also request that the robot speaks in response to detecting the object
            greeting = userdata.mission_data + " detected"
            rospy.loginfo(greeting)
            self.__helper_obj.Speak(greeting, greeting)
        else:
            next_outcome = 'move_camera'
        
        return next_outcome                      
 
 
# Helper class for object search
class ObjectSearchHelper():
    def __init__(self):
        self.__object_detection_request_pub = rospy.Publisher('tf_object_detection_node/start',
                                                              Empty, queue_size=1)

    def MovementCompleteOS(self, userdata, msg):
        # Camera movement is complete

        # Request the object detection
        object_detection_request = Empty()
        self.__object_detection_request_pub.publish(object_detection_request)         
        
        # Returning False means the state transition will follow the invalid outcome
        return False
        
    def AnalysisCompleteOS(self, userdata, msg):
        # Analysis for object detection complete
        
        # Extract the result results
        if len(msg.names_detected) == 0:
            # No objects recognised
            userdata.object_detected = False
        else:
            # Objects detected, but did we find the one we are interested in?
            userdata.object_detected = False
            for n in msg.names_detected:
                if n == userdata.mission_data:
                    userdata.object_detected = True
                    break
                            
        # Returning False means the state transition will follow the invalid outcome
        return False
        
    def UserAckOS(self, userdata, msg):
        # User acknowledged so move on
        
        # Set start to False so the next state knows this is a continuation of
        # the mission and not a new mission
        userdata.start = False
        
        # Returning False means the state transition will follow the invalid outcome
        return False
