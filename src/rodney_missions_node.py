#!/usr/bin/env python
# Copyright 2019 Philip Hopley
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a  copy
# of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
#
# This ROS node is the state machine for the missions that control Rondey Robot
import sys
import rospy
import roslib; roslib.load_manifest('std_srvs')
import std_srvs.srv
import missions_lib
from std_msgs.msg import String, Empty, Bool
from smach import State, StateMachine
from smach_ros import MonitorState, SimpleActionState, IntrospectionServer
from actionlib_msgs.msg import GoalStatus
from speech.msg import voice
from head_control.msg import point_headAction, point_headGoal
from math import pi

# Helper class to hold code used by serveral different states
class MissionsHelper():
    def __init__(self):
        self.__speech_pub_ = rospy.Publisher('/speech/to_speak', voice, queue_size=1)
        self.__text_out_pub = rospy.Publisher('/robot_face/text_out', String, queue_size=1)
        
        # Obtain values from the parameter server
        # Minium/Maximum range movment of camera
        self.__pan_min = rospy.get_param("/servo/index0/pan/min", -(pi/2.0))
        self.__pan_max = rospy.get_param("/servo/index0/pan/max", pi/2.0)
        self.__tilt_min = rospy.get_param("/servo/index0/tilt/min", -(pi/2.0))
        self.__tilt_max = rospy.get_param("/servo/index0/tilt/max", pi/2.0);
        # Default position after mission ends
        self.__camera_default_pan_position = rospy.get_param("/head/position/pan", 0.0)
        self.__camera_default_tilt_position = rospy.get_param("/head/position/tilt", 0.0)
        # Using '~private_name' will prefix the parameters with the node name given in launch file
        # Step value to move the camera by when searching
        self.__pan_step_value = rospy.get_param("~head/view_step/pan", 0.436332)
        self.__tilt_step_value = rospy.get_param("~head/view_step/tilt", 0.436332)
        # Step value to move the camera in manual mode
        self.__manual_pan_step_value = rospy.get_param("~head/manual_view_step/pan", 0.174533)
        self.__manual_tilt_step_value = rospy.get_param("~head/manual_view_step/tilt", 0.174533)
        # Position to move to for user input
        self.__user_input_position_pan = rospy.get_param("~head/user_position/pan", 0.0)
        self.__user_input_position_tilt = rospy.get_param("~head/user_position/tilt", -0.5)
        
        # When true and scanning pan angle will increase, otherwise decrease
        self.__increase_pan = True
        
        # Position that will be requested to move the head/camera to
        self.__position_request_pan = self.__camera_default_pan_position   
        self.__position_request_tilt = self.__camera_default_tilt_position

        # RPLidar services to start and stop the motor
        rospy.wait_for_service('stop_motor')
        rospy.wait_for_service('start_motor')
        self.__rplidar_stop_motor_srv = rospy.ServiceProxy('stop_motor', std_srvs.srv.Empty)
        self.__rplidar_start_motor_srv = rospy.ServiceProxy('start_motor', std_srvs.srv.Empty)
        # LIDAR should be running but make sure
        self.LidarEnable()        
 
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

    # Function to enable the RPLidar
    def LidarEnable(self):        
        self.__rplidar_start_motor_srv()
        self.__lidar_on = True
        
    # Function to disable the RPLidar
    def LidarDisable(self):        
        self.__rplidar_stop_motor_srv() 
        self.__lidar_on = False 
        
    # Function to Toggle RPLidar on/off
    def ToggleLidar(self):
        if(self.__lidar_on == True):
            self.LidarDisable()
        else:
            self.LidarEnable()     
    
    # Function to return the camera start position when scanning within head movement range         
    def CameraToStartPos(self):
        # Set the camera position to pan min and tilt min
        self.__position_request_pan = self.__pan_min
        self.__position_request_tilt = self.__tilt_max
        # Set the variable that says which direction the pan is going. Start by incrementing
        self.__increase_pan_ = True
        return self.__position_request_pan, self.__position_request_tilt 

    # Function to keep track of position after action to set to default position
    def CameraAtDefaultPos(self, userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            self.__position_request_pan = self.__camera_default_pan_position
            self.__position_request_tilt = self.__camera_default_tilt_position
        
    # Function returns camera default position
    def CameraDefaultPos(self):
        return self.__camera_default_pan_position, self.__camera_default_tilt_position           
 
    # Function to return the next position when scanning within the head movement range.
    # Also returns indication if all areas scanned or more left           
    def CameraToNextPos(self):
        all_areas_scanned = False
        # Calculate the next position of the head/camera        
        if self.__increase_pan == True:
            if self.__position_request_pan == self.__pan_max:
                # Last scan was at the edge, move tilt up and then pan the other way
                self.__increase_pan = False
                self.__position_request_tilt -= self.__tilt_step_value
                  
                if self.__position_request_tilt < self.__tilt_min:
                    all_areas_scanned = True
            else:
                self.__position_request_pan += self.__pan_step_value
                    
                if self.__position_request_pan > self.__pan_max:
                    # Moved out of range, put back on max
                    self.__position_request_pan = self.__pan_max
        else:    
            if self.__position_request_pan == self.__pan_min:
                # Last scan was at the edge, move tilt up and then pan the other way
                self.__increase_pan = True
                self.__position_request_tilt -= self.__tilt_step_value
                    
                if self.__position_request_tilt < self.__tilt_min:
                    all_areas_scanned = True
            else:
                self.__position_request_pan -= self.__pan_step_value
                    
                if self.__position_request_pan < self.__pan_min:
                    # Moved out of range, put back on min
      	            self.__position_request_pan = self.__pan_min 
      	            
      	if all_areas_scanned == True:
      	    # Reset camera/head position to default values                
            self.__position_request_pan = self.__camera_default_pan_position
            self.__position_request_tilt = self.__camera_default_tilt_position                      
     
        return all_areas_scanned, self.__position_request_pan, self.__position_request_tilt
        
    def CameraManualMove(self, direction):
        relative_request_pan = 0.0
        relative_request_tilt = 0.0
        # Check for up command
        if 'd' in direction:
            relative_request_tilt = self.__manual_tilt_step_value
            if (self.__position_request_tilt + relative_request_tilt) > self.__tilt_max:
                # Would move out of range so move to the max position                   
	            relative_request_tilt = self.__tilt_max - self.__position_request_tilt
	            self.__position_request_tilt = self.__tilt_max
            else:
                # Keep track
                self.__position_request_tilt += relative_request_tilt
                    
        # Check for down command
        if 'u' in direction:
            relative_request_tilt = -(self.__manual_tilt_step_value)
            if (self.__position_request_tilt + relative_request_tilt) < self.__tilt_min:
                # Would move out of range so move to the min position                 
	            relative_request_tilt = self.__tilt_min - self.__position_request_tilt
	            self.__position_request_tilt = self.__tilt_min
            else:
                # Keep track
                self.__position_request_tilt += relative_request_tilt
            
        # Check for left commnand
        if 'l' in direction:
            relative_request_pan = self.__manual_pan_step_value
            if (self.__position_request_pan + relative_request_pan) > self.__pan_max:
                # Would move out of range so move to the max                   
	            relative_request_pan = self.__pan_max - self.__position_request_pan
	            self.__position_request_pan = self.__pan_max
            else:
                # Keep track
                self.__position_request_pan += relative_request_pan
            
        # Check for right command
        if 'r' in direction:

            relative_request_pan = -(self.__manual_pan_step_value)
            if (self.__position_request_pan + relative_request_pan) < self.__pan_min:
                # Would move out of range so so move to min                   
	            relative_request_pan = self.__pan_min - self.__position_request_pan
	            self.__position_request_pan = self.__pan_min
            else:
                # Keep track
                self.__position_request_pan += relative_request_pan            
            
        return relative_request_pan, relative_request_tilt
        
    def UserInputPosition(self):
        return self.__user_input_position_pan, self.__user_input_position_tilt               
    
                          
# The PREPARE state
class Prepare(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['mission1','mission2','mission4','done_task','head_default','move_head'], 
                       input_keys=['mission'],
                       output_keys=['mission_data','start','user_data_absolute','user_data_pan','user_data_tilt'])
        self.__helper_obj = helper_obj
    
    def execute(self, userdata):        
        # Based on the userdata either change state to the required mission or 
        # carry out single job userdata.mission contains the mission or single 
        # job and a number of parameters seperated by '^'
        retVal = 'done_task';
        
        # Split into parameters using '^' as the delimiter
        parameters = userdata.mission.split("^")
        
        if parameters[0] == 'M1':
            # Mission 1 to search for a known person and deliver a message
            # parameter[1] contains filename of file containing poses for the patrol,
            # the id of the person to search for and the message to deliver            
            userdata.mission_data = parameters[1]
            retVal = 'mission1'        
        elif parameters[0] == 'M2':
            # Mission 2 is scan for faces and greet those known, there are no
            # other parameters with this mission request
            userdata.start = True
            retVal = 'mission2'
        elif parameters[0] == 'M4':            
            # Mission 4 is go home. parameter[1] contains filename of file containing poses
            # It may also contain a waypoint to got to. If not we will go
            userdata.mission_data = parameters[1]            
            retVal = 'mission4'
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
            # be sent in manual mode.
            # parameters[1] will either be 'u', 'd', 'c', 'i' or '-'
            # parameters[2] will either be 'l', 'r' or '-'
            # Check for command
            if 'c' in parameters[1]:
                # Move to default position
                retVal = 'head_default'
            elif 'i' in parameters[1]:
                # Move to user input position. This position is a good position for user input at the screen
                pan_position, tilt_position = self.__helper_obj.UserInputPosition()
                userdata.user_data_absolute = True # This will be a actual position move
                userdata.user_data_pan = pan_position
                userdata.user_data_tilt = tilt_position
                retVal = 'move_head'
            else:                
                relative_request_pan, relative_request_tilt = self.__helper_obj.CameraManualMove(parameters[1]+parameters[2])                
                # Set up user data that will be used for goal in next state
                userdata.user_data_absolute = False # This will be a relative move
                userdata.user_data_pan = relative_request_pan
                userdata.user_data_tilt = relative_request_tilt
                retVal = 'move_head'                
        elif parameters[0] == 'J4':
            # Simple job to toggle the LIDAR on/off
            self.__helper_obj.ToggleLidar()

        return retVal
        

# The REPORT state
class Report(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['mission_status'])
        self.__pub = rospy.Publisher('/missions/mission_complete', String, queue_size=5)
    
    def execute(self, userdata):        
        # Publishes message that mission completed
        message = userdata.mission_status
        self.__pub.publish(message)
        return 'success'        
        
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
        
        # ------------------------- Top level state machine -------------------------
        # Create top level state machine
        self.__sm = StateMachine(outcomes=['preempted','aborted'],                                 
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
                             transitions={'mission1':'MISSION1','mission2':'MISSION2','mission4':'MISSION4',
                                          'done_task':'WAITING','head_default':'DEFAULT_HEAD_POSITION',
                                          'move_head':'MOVE_HEAD'})
                             
            # Add the reporting state
            StateMachine.add('REPORT',
                             Report(),
                             transitions={'success':'DEFAULT_HEAD_POSITION'})

            # Set up action goal for deafult head position            
            default_position_pan, default_position_tilt = self.__missions_helper.CameraDefaultPos()
            head_goal = point_headGoal()
            head_goal.absolute = True
            head_goal.pan = default_position_pan
            head_goal.tilt = default_position_tilt

            # Add the default camera position state. Which moves the head to the default position
            StateMachine.add('DEFAULT_HEAD_POSITION',
                             SimpleActionState('head_control_node',
                                               point_headAction,
                                               result_cb = self.__missions_helper.CameraAtDefaultPos,
                                               goal = head_goal),
                             transitions={'succeeded':'WAITING','preempted':'WAITING','aborted':'aborted'})

            # Add the move head state
            StateMachine.add('MOVE_HEAD',
                             SimpleActionState('head_control_node',
                                               point_headAction,
                                               goal_slots=['absolute', 'pan', 'tilt']),
                             transitions={'succeeded':'WAITING', 'preempted':'WAITING', 'aborted':'aborted'},
                             remapping={'absolute':'user_data_absolute', 'pan':'user_data_pan', 'tilt':'user_data_tilt'}) 

            # ------------------------- Sub State machine for mission 1 ---------------------
            # Create a sub state machine for mission 1 - take a message to
            self.__sm_mission1 = missions_lib.Mission1StateMachine(self.__missions_helper)
            
            # Now add the sub state machine for mission 1 to the top level one
            StateMachine.add('MISSION1', 
                             self.__sm_mission1, 
                             transitions={'complete':'REPORT','preempted':'REPORT','aborted':'REPORT'}) 
            
            # ------------------------- Sub State machine for mission 2 ---------------------
            # Create a sub state machine for mission 2 - face detection and greeting
            self.__sm_mission2 = missions_lib.Mission2StateMachine(self.__missions_helper)

            # Now add the sub state machine for mission 2 to the top level one
            StateMachine.add('MISSION2', 
                             self.__sm_mission2, 
                             transitions={'complete':'REPORT','preempted':'REPORT','aborted':'aborted'}) 
                                                                    
            # ------------------------- Sub State machine for mission 4 ---------------------
            # Create a sub state machine for mission 4 - Go Home
            self.__sm_mission4 = missions_lib.Mission4StateMachine(self.__missions_helper)
            
            # Now add the sub state machine for mission 4 to the top level one
            StateMachine.add('MISSION4', 
                             self.__sm_mission4, 
                             transitions={'complete':'REPORT','preempted':'REPORT','aborted':'REPORT'})            
            # -------------------------------------------------------------------------------
            
            
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
        userdata.mission = msg.data       
                        
        # Returning False means the state transition will follow the invalid line
        return False
        
    # Callback for cancel mission message
    def CancelCallback(self, data):
        # If a sub statemachine for a mission is running then request it be preempted
        if self.__sm_mission1.is_running():
            self.__sm_mission1.request_preempt() 
        elif self.__sm_mission2.is_running():
            self.__sm_mission2.request_preempt() 
        elif self.__sm_mission4.is_running():
            self.__sm_mission4.request_preempt()                   
        
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

