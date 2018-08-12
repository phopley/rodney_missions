#!/usr/bin/env python
# States for search for people by moving the head only and to greet people that it sees who are known

import rospy
from smach import State

# Greeting State
class Greeting(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                       input_keys=['detected'])
    
    def execute(self, userdata):        
        # userdata.detected.ids_detected is the IDs of those detected
        # userdata.detected.names_detected is the name of thise detected
        
        # Construct greeting
        greeting = ''
        if len(userdata.detected.names_detected) == 0:
            greeting = 'No one recognised'
        else:
            greeting = 'Hello '
            for n in userdata.detected.names_detected:
                greeting += n + ' '
                
            greeting += 'how are you '
            
            if len(userdata.detected.names_detected) == 1:
                greeting += 'today'
            elif len(userdata.detected.names_detected) == 2:
                greeting += 'both'
            else:
                greeting += 'all'
            
        rospy.loginfo(greeting)
        
        return 'success'
        
