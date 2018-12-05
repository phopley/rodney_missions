#!/usr/bin/env python
# This ROS node is the state machine for the missions

import sys
import rospy
import missions_lib
from std_msgs.msg import String, Empty
from servo_msgs.msg import pan_tilt
from face_recognition_msgs.msg import face_recognition
from tf_object_detection.msg import detection_results
from smach import State, StateMachine
from smach_ros import MonitorState, IntrospectionServer
from speech.msg import voice

# Helper class to hold code used by serveral different states
class MissionsHelper():
    def __init__(self):
        self.__camera_position_request_pub = rospy.Publisher('/head_control_node/head_position_request',
                                                             pan_tilt, queue_size=1)
        self.__speech_pub_ = rospy.Publisher('/speech/to_speak', voice, queue_size=1)
        self.__text_out_pub = rospy.Publisher('/robot_face/text_out', String, queue_size=1)
        
        # Obtain values from the parameter server
        # Minium/Maximum range movment of camera
        self.__pan_min = rospy.get_param("/servo/index0/pan_min", 0)
        self.__pan_max = rospy.get_param("/servo/index0/pan_max", 180)
        self.__tilt_min = rospy.get_param("/servo/index0/tilt_min", 0)
        self.__tilt_max = rospy.get_param("/servo/index0/tilt_max", 180);
        # Default position after mission ends
        self.__camera_default_pan_position = rospy.get_param("/head/position/pan", 90)
        self.__camera_default_tilt_position = rospy.get_param("/head/position/tilt", 45)
        # Step value to move the camera by when searching
        self.__pan_step_value = rospy.get_param("/head/view_step/pan", 10)
        self.__tilt_step_value = rospy.get_param("/head/view_step/tilt", 10)
        
        # When true and scanning pan angle will increase, otherwise decrease
        self.__increase_pan = True
        
        # Position that will be requested to move the head/camera to
        self.__position_request = pan_tilt()
        self.__position_request.pan = self.__camera_default_pan_position   
        self.__position_request.tilt = self.__camera_default_tilt_position
 
    def Speak(self, text_to_speak, text_to_display):
        voice_msg = voice()
        voice_msg.text = text_to_speak
        voice_msg.wav = ""
        
        # Publish topic for speech and robot face animation
        self.__speech_pub_.publish(voice_msg)
        self.__text_out_pub.publish(text_to_display)
    
    def Wav(self, wav_file, text_to_display):
        voice_msg = voice()
        voice_msg.text = ""
        voice_msg.wav = wav_file
        
        # Publish
        self.__speech_pub_.publish(voice_msg)
        self.__text_out_pub.publish(text_to_display)
         
    def CameraToStartPos(self):
        # Set the camera position to pan min and tilt min
        self.__position_request.pan = self.__pan_min
        self.__position_request.tilt = self.__tilt_min
        # Set the variable that says which direction the pan is going. Start by incrementing
        self.__increase_pan_ = True
        # Send message to move the camera
        self.__camera_position_request_pub.publish(self.__position_request)

    def CameraToDefaultPos(self):
        self.__position_request.pan = self.__camera_default_pan_position
        self.__position_request.tilt = self.__camera_default_tilt_position
        # Send message to move the camera
        self.__camera_position_request_pub.publish(self.__position_request)
            
    def CameraToNextPos(self):
        all_areas_scanned = False
        # Calculate the next position of the head/camera        
        if self.__increase_pan == True:
            if self.__position_request.pan == self.__pan_max:
                # Last scan was at the edge, move tilt up and then pan the other way
                self.__increase_pan = False                    
                self.__position_request.tilt += self.__tilt_step_value
                  
                if self.__position_request.tilt > self.__tilt_max:
                    all_areas_scanned = True
            else:
                self.__position_request.pan += self.__pan_step_value
                    
                if self.__position_request.pan > self.__pan_max:
                    # Moved out of range, put back on max
                    self.__position_request.pan = self.__pan_max
        else:    
            if self.__position_request.pan == self.__pan_min:
                # Last scan was at the edge, move tilt up and then pan the other way
                self.__increase_pan = True
                self.__position_request.tilt += self.__tilt_step_value
                    
                if self.__position_request.tilt > self.__tilt_max:
                    all_areas_scanned = True
            else:
                self.__position_request.pan -= self.__pan_step_value
                    
                if self.__position_request.pan < self.__pan_min:
                    # Moved out of range, put back on min
      	            self.__position_request.pan = self.__pan_min 
      	            
      	if all_areas_scanned == True:
      	    # Reset camera/head position to default values                
            self.__position_request.pan = self.__camera_default_pan_position
            self.__position_request.tilt = self.__camera_default_tilt_position
                    
        # Send message to move the camera
        self.__camera_position_request_pub.publish(self.__position_request)
       
        return all_areas_scanned
        
    def CameraManualMove(self, direction):
        # Check for up command
        if 'u' in direction:
            self.__position_request.tilt += self.__tilt_step_value
            if self.__position_request.tilt > self.__tilt_max:
                # Moved out of range, put back on max                   
	            self.__position_request.tilt = self.__tilt_max;
                    
        # Check for down command
        if 'd' in direction:
            self.__position_request.tilt -= self.__tilt_step_value
            if self.__position_request.tilt < self.__tilt_min:
                # Moved out of range, put back on min                  
	            self.__position_request.tilt = self.__tilt_min;
            
        # Check for left commnand
        if 'l' in direction:
            self.__position_request.pan += self.__pan_step_value
            if self.__position_request.pan > self.__pan_max:
                # Moved out of range, put back on max                   
	            self.__position_request.pan = self.__pan_max;
            
        # Check for right command
        if 'r' in direction:
            self.__position_request.pan -= self.__pan_step_value
            if self.__position_request.pan < self.__pan_min:
                # Moved out of range, put back on min                  
	            self.__position_request.pan = self.__pan_min;
            
        # Check for return to default command
        if 'c' in direction:
            self.__position_request.pan = self.__camera_default_pan_position
            self.__position_request.tilt = self.__camera_default_tilt_position
            
        # Send message to move the camera
        self.__camera_position_request_pub.publish(self.__position_request)
    
                          
# The PREPARE state
class Prepare(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['mission2','mission3','done_task'], 
                       input_keys=['mission'],
                       output_keys=['mission_data','start'])
        self.__helper_obj = helper_obj
    
    def execute(self, userdata):        
        # Based on the userdata either change state to the required mission or 
        # carry out single job userdata.mission contains the mission or single 
        # job and a number of parameters seperated by '^'
        retVal = 'done_task';
        
        # Split into parameters using '^' as the delimiter
        parameters = userdata.mission.split("^")
        
        if parameters[0] == 'M2':
            # Mission 2 is scan for faces and greet those known, there are no
            # other parameters with this mission request
            userdata.start = True
            retVal = 'mission2'
        elif parameters[0] == 'M3':
            # Mission 3 is to scan for the given object
            # parameters[1] will contain the object to detect
            userdata.mission_data = parameters[1]
            userdata.start = True
            retVal = 'mission3'
        elif parameters[0] == 'J1':
            # Simple Job 1 is play a supplied wav file and move the face lips           
            # Publish topic for speech wav and robot face animation
            self.__helper_obj.Wav(parameters[1], parameters[2])
        elif parameters[0] == 'J2':
            # Simple Job 2 is to speak the supplied text and move the face lips
            # Publish topic for speech and robot face animation
            self.__helper_obj.Speak(parameters[1], parameters[2])
        elif parameters[0] == 'J3':
            # Simple Job 3 is to move the head/camera. This command will only
            # be sent in manual mode. The resultant published message will only
            # be sent once per received command.
            # parameters[1] will either be 'u', 'd', 'c' or '-'
            # parameters[2] will either be 'l', 'r' or '-'             
            self.__helper_obj.CameraManualMove(parameters[1]+parameters[2])
        return retVal
        
# The REPORT state
class Report(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.__pub = rospy.Publisher('/missions/mission_complete', String, queue_size=5)
    
    def execute(self, userdata):        
        # Publishes message that mission completed
        self.__pub.publish("Mission Complete")
        return 'success'        

# The SUB_PREEMPTED state
class SubPreempted(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete'])
        self.__helper_obj = helper_obj
        
    def execute(self, userdata):
        # Reset camera to default position
        self.__helper_obj.CameraToDefaultPos()
        return 'complete'
        
# Top level state machine. The work for each mission is another state machine
# in the 'mission' states        
class RodneyMissionsNode:
    def __init__(self):
        rospy.on_shutdown(self.ShutdownCallback)
        
        # Subscribe to message to cancel missions        
        self.__cancel_sub = rospy.Subscriber('/missions/mission_cancel', Empty, 
                                             self.CancelCallback)        
        
        # Create an instance of the missions helper class
        self.__missions_helper = MissionsHelper()
        
        # Create an instance of the helper class for greeting
        self.__greet_helper = missions_lib.GreetingHelper()
        
        # Create an instance of the helper class for object search
        self.__os_helper = missions_lib.ObjectSearchHelper()
        
        # ------------------------- Top level state machine -------------------------
        # Create top level state machine
        self.__sm = StateMachine(outcomes=['preempted'],
                                 output_keys=['mission_data'])
        with self.__sm:                                   
            # Add a state which monitors for a mission to run            
            StateMachine.add('WAITING',
                             MonitorState('/missions/mission_request',
                             String,
                             self.MissionRequestCB,                             
                             output_keys = ['mission']),
                             transitions={'valid':'WAITING', 'invalid':'PREPARE', 
                                          'preempted':'preempted'})
                             
            # Add state to prepare the mission            
            StateMachine.add('PREPARE',
                             Prepare(self.__missions_helper),                             
                             transitions={'mission2':'MISSION2','mission3':'MISSION3',
                                          'done_task':'WAITING'})
                             
            # Add the reporting state
            StateMachine.add('REPORT',
                             Report(),
                             transitions={'success':'WAITING'})

            # Add the sub-state preemeted state
            StateMachine.add('SUB_PREEMPTED',
                             SubPreempted(self.__missions_helper),
                             transitions={'complete':'WAITING'})            


            # ------------------------- State machine for mission 2 -------------------------
            # Create a sub state machine for mission 2 - greeting
            self.__sm_mission2 = StateMachine(outcomes=['complete', 'preempted'],
                                              input_keys=['start'])

            with self.__sm_mission2:
                # This state will move the camera                               
                StateMachine.add('MOVE_CAMERA_GRT',
                                 missions_lib.MoveCameraGeeting(self.__missions_helper),
                                 transitions={'complete':'GREETING',
                                              'awaiting_movement':'WAITING_FOR_MOVEMENT_GRT'},
                                 remapping={'start_in':'start','start_out':'start'})
                                           
                # This state will wait until the camera is moved and then 
                # request check for face recognition on the camera image
                StateMachine.add('WAITING_FOR_MOVEMENT_GRT',
                                 MonitorState('/head_control_node/head_move_complete',
                                              Empty,
                                              self.__greet_helper.MovementCompleteGrt),
                                 transitions={'valid':'WAITING_FOR_MOVEMENT_GRT',
                                              'invalid':'WAITING_FOR_ANALYSIS_GRT',
                                              'preempted':'preempted'})

                # This state will wait until face recognition is run on the
                # image and the results returned
                StateMachine.add('WAITING_FOR_ANALYSIS_GRT',
                                 MonitorState('/face_recognition_node/result',
                                              face_recognition,
                                              self.__greet_helper.AnalysisCompleteGrt,
                                              input_keys = ['seen_dict_in'],
                                              output_keys = ['seen_dict_out']),
                                 remapping={'seen_dict_in':'seen_dict',
                                            'seen_dict_out':'seen_dict'},
                                 transitions={'valid':'WAITING_FOR_ANALYSIS_GRT',
                                              'invalid':'MOVE_CAMERA_GRT',
                                              'preempted':'preempted'}) 
                                                                      
                StateMachine.add('GREETING',
                                 missions_lib.Greeting(self.__missions_helper),
                                 transitions={'complete':'complete'})
                                             
            # Now add the sub state machine for mission 3 to the top level one
            StateMachine.add('MISSION2', 
                             self.__sm_mission2, 
                             transitions={'complete':'REPORT', 'preempted':'SUB_PREEMPTED'})
                                         
            # ------------------------- State machine for mission 3 -------------------------        
            # Create a sub state machine for mission 3 - object search
            self.__sm_mission3 = StateMachine(outcomes=['complete', 'preempted'],
                                              input_keys=['mission_data','start'])
            
            with self.__sm_mission3:
                # This state will move the camera
                StateMachine.add('MOVE_CAMERA_OS',
                                 missions_lib.MoveCameraObjectSearch(self.__missions_helper),
                                 transitions={'awaiting_movement':'WAITING_FOR_MOVEMENT_OS',
                                              'complete':'complete'},
                                 remapping={'start_in':'start','start_out':'start'})
                                 
                # This state will wait until the camera is moved and then
                # request to detect objects from the camera image
                StateMachine.add('WAITING_FOR_MOVEMENT_OS',
                                 MonitorState('/head_control_node/head_move_complete',
                                 Empty,
                                 self.__os_helper.MovementCompleteOS),
                                 transitions={'valid':'WAITING_FOR_MOVEMENT_OS', 
                                              'invalid':'WAITING_FOR_ANALYSIS_OS',
                                              'preempted':'preempted'})
                
                # This state will wait until object detection is run on an
                # image and the results returned
                StateMachine.add('WAITING_FOR_ANALYSIS_OS',
                                 MonitorState('/tf_object_detection_node/result',
                                 detection_results,
                                 self.__os_helper.AnalysisCompleteOS,
                                 input_keys = ['mission_data'],
                                 output_keys = ['object_detected']),
                                 transitions={'valid':'WAITING_FOR_ANALYSIS_OS', 
                                              'invalid':'CHOICE_OS',
                                              'preempted':'preempted'}) 
                
                # This state will will make the decision which state to move to
                # next. If the object in question was detected then it will move
                # to a state that waits for user interaction. If the object was
                # not detected then it will move to the state to move the camera.
                # This seems a bit of an overkill but the MonitorState before it
                # can only provide the three possible outcomes 'valid', 'invalid'
                # or 'preempted'. This state will request speech to state the
                # object was found.
                StateMachine.add('CHOICE_OS',
                                 missions_lib.ChoiceObjectSearch(self.__missions_helper),           
                                 transitions= {'move_camera':'MOVE_CAMERA_OS',
                                               'await_user':'WAIT_FOR_USER_OS'})
                
                # This state will wait for a message indicating the user wishes
                # to move on. It will allow the user to examine the camera feed
                # once an object was detected
                StateMachine.add('WAIT_FOR_USER_OS',
                                 MonitorState('/missions/acknowledge',
                                 Empty,
                                 self.__os_helper.UserAckOS),
                                 transitions={'valid':'WAIT_FOR_USER_OS',
                                              'invalid':'MOVE_CAMERA_OS',
                                              'preempted':'preempted'})
                                                            
            # Now add the sub state machine for mission 3 to the top level one
            StateMachine.add('MISSION3', 
                             self.__sm_mission3, 
                             transitions={'complete':'REPORT', 'preempted':'SUB_PREEMPTED'}) 
            
        # Create and start the introspective server so that we can use smach_viewer
        sis = IntrospectionServer('server_name', self.__sm, '/SM_ROOT')
        sis.start()
                             
        self.__sm.execute()
        
        # Wait for ctrl-c to stop application
        rospy.spin()
        sis.stop()
        
    
    # Monitor State takes /missions/mission_request topic and passes the mission
    # in user_data to the PREPARE state
    def MissionRequestCB(self, userdata, msg):                
        # Take the message data and send it to the next state in the userdata
        userdata.mission = msg.data;       
                        
        # Returning False means the state transition will follow the invalid line
        return False
        
    # Callback for cancel mission message
    def CancelCallback(self, data):
        # If a sub statemachine for a mission is running then request it be preempted
        if self.__sm_mission2.is_running():
            self.__sm_mission2.request_preempt()
        elif self.__sm_mission3.is_running():
           
            self.__sm_mission3.request_preempt()        
        
    def ShutdownCallback(self):        
        self.__sm.request_preempt()
        # Although we have requested to shutdown the state machine 
        # it will not happen if we are in WAITING until a message arrives
        
def main(args):
    rospy.init_node('rodney_missions_node', anonymous=False)
    rospy.loginfo("Rodney missions node started")
    rmn = RodneyMissionsNode()        

if __name__ == '__main__':
    main(sys.argv)

