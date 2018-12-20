#!/usr/bin/env python
# States for searching for an object

import rospy
from smach import State
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Empty
from servo_msgs.msg import pan_tilt


# PREPARE_FOR_MOVEMENT_OS State
class PrepareMovementObjectSearch(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete','move'],
                       input_keys=['start_in'],
                       output_keys=['start_out','user_data_absolute','user_data_pan','user_data_tilt'])
        self.__helper_obj = helper_obj                                 
                       
    def execute(self, userdata):
        position_request = pan_tilt()

        # Is this a new search?
        if userdata.start_in == True:
            userdata.start_out = False
            scan_complete = False
            # get the camera start position (pan min and tilt min)
            position_request = self.__helper_obj.CameraToStartPos()
        else:            
            scan_complete, position_request = self.__helper_obj.CameraToNextPos()

        # Set up user data that will be used for goal in next state if not complete
        userdata.user_data_absolute = True
        userdata.user_data_pan = position_request.pan
        userdata.user_data_tilt = position_request.tilt

        if scan_complete == True:
            greeting = "Mission complete Rodney standing by"
            # Speak
            self.__helper_obj.Speak(greeting, greeting)
            next_outcome = 'complete'
        else:
            next_outcome = 'move'                
        	    
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

    def MovementCompleteCB(self, userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Camera movement is complete
            # Request the object detection
            object_detection_request = Empty()
            self.__object_detection_request_pub.publish(object_detection_request)         
        
        
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
        
        # Returning False means the state transition will follow the invalid outcome
        return False
