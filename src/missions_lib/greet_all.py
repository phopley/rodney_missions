#!/usr/bin/env python
# States for search for people by moving the head only and to greet people
# that it sees who are known

import rospy
from std_msgs.msg import Empty
from smach import State

# MOVE_CAMERA_GRT State
class MoveCameraGeeting(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete','awaiting_movement'],
                       input_keys=['start_in'],
                       output_keys=['start_out','seen_dict'])
        self.__helper_obj = helper_obj
                                                             
    def execute(self, userdata):
        # Is this the start of a new mission
        if userdata.start_in == True:
            userdata.start_out = False
            # clear the seen dictionary
            userdata.seen_dict = {}
            scan_complete = False
            # Set the camera position to start position (pan min and tilt min)
            self.__helper_obj.CameraToStartPos()
        else:            
            scan_complete = self.__helper_obj.CameraToNextPos()
        
        if scan_complete == True:
            next_outcome = 'complete'
        else:
            next_outcome = 'awaiting_movement'
        
        return next_outcome                                                            

# Greeting State
class Greeting(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete'],
                       input_keys=['seen_dict'])
        self.__helper_obj = helper_obj

    def execute(self, userdata):
        # userdata.seen_dict contains a dictionary of ids and names seen
        # Construct greeting
        greeting = ''
        if len(userdata.seen_dict) == 0:
            greeting = 'No one recognised'
        else:
            greeting = 'Hello '
            for id, name in userdata.seen_dict.items():
                greeting += name + ' '
                
            greeting += 'how are you '
            
            if len(userdata.seen_dict) == 1:
                greeting += 'today'
            elif len(userdata.seen_dict) == 2:
                greeting += 'both'
            else:
                greeting += 'all'
        
        rospy.loginfo(greeting)
            
        # Speak greeting
        self.__helper_obj.Speak(greeting, greeting + ':)')
        
        return 'complete'


# Helper class for greeting
class GreetingHelper():
    def __init__(self):
        self.__scan_request_pub = rospy.Publisher('face_recognition_node/start',
                                                  Empty, queue_size=1)

    def MovementCompleteGrt(self, userdata, msg):
        # Camera movement is complete

        # Request the face recognition
        face_recognition_request = Empty()
        self.__scan_request_pub.publish(face_recognition_request)         
        
        # Returning False means the state transition will follow the invalid outcome
        return False
        
    def AnalysisCompleteGrt(self, userdata, msg):
        # Analysis for face recognition is complete
        local_dict = userdata.seen_dict_in
        if len(msg.ids_detected) > 0:
            # Recognised faces detected. Have we seen then before or are they new
            for idx, val in enumerate(msg.ids_detected):
                if val not in local_dict:
                    # Add to dictionary
                    local_dict[val] = msg.names_detected[idx]
                    
                    # Log who was seen                                     
                    rospy.loginfo("Greeting: I have seen %s", msg.names_detected[idx])
            
            # Update disctionary stored in user data        
            userdata.seen_dict_out = local_dict
                                                 
        # Returning False means the state transition will follow the invalid outcome
        return False
