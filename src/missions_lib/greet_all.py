#!/usr/bin/env python
# States for search for people by moving the head only and to greet people that it sees who are known

import rospy
from std_msgs.msg import String
from smach import State
from speech.msg import voice

# Greeting State
class Greeting(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'],
                       input_keys=['detected'])
        self.__speech_pub_ = rospy.Publisher('/speech/to_speak', voice, queue_size=5)
        self.__text_out_pub = rospy.Publisher('/robot_face/text_out', String, queue_size=5)        
    
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
        
        voice_msg = voice()
        voice_msg.text = greeting
        voice_msg.wav = ""
        
        # Publish topic for speech and robot face animation
        self.__speech_pub_.publish(voice_msg)
        self.__text_out_pub.publish(greeting + ":)")
        
        return 'success'
        
