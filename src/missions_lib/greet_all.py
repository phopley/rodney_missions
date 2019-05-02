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
# States for search for people by moving the head only and to greet people
# that it sees who are known
import rospy
from smach import State, StateMachine
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus
from head_control.msg import point_headAction, point_headGoal
from face_recognition_msgs.msg import scan_for_facesAction

# Child (derived) class. Parent class is StateMachine
class Mission2StateMachine(StateMachine):
    def __init__(self, helper_obj):
        StateMachine.__init__(self, outcomes=['complete','preempted','aborted'], input_keys=['start'], output_keys=['mission_status'])

        self.__helper_obj = helper_obj

        with self:
            # This state will calculate the next head/camera position
            StateMachine.add('PREPARE_FOR_MOVEMENT_GRT',
                             PrepareMovementGeeting(self.__helper_obj),
                             transitions={'complete':'GREETING','move':'MOVE_HEAD_GRT'},
                             remapping={'start_in':'start','start_out':'start'})
 
            # This state will call the action to move the head/camera
            StateMachine.add('MOVE_HEAD_GRT',
                             SimpleActionState('head_control_node',
                                               point_headAction,                                                  
                                               goal_slots=['absolute','pan','tilt'],
                                               result_cb = self.moveHeadComplete,
                                               output_keys=['mission_status']),
                             transitions={'succeeded':'SCAN_FOR_FACES','preempted':'preempted','aborted':'aborted'},
                             remapping={'absolute':'user_data_absolute','pan':'user_data_pan','tilt':'user_data_tilt'})                                          

            # This state will call the action to scan for faces on the image from the camera
            StateMachine.add('SCAN_FOR_FACES',
                             SimpleActionState('face_recognition',
                                               scan_for_facesAction,
                                               result_cb=self.face_recognition_result_cb,
                                               input_keys=['seen_dict_in'],
                                               output_keys=['seen_dict_out','mission_status']),
                              remapping={'seen_dict_in':'seen_dict','seen_dict_out':'seen_dict'},                      
                              transitions={'succeeded':'PREPARE_FOR_MOVEMENT_GRT','preempted':'preempted','aborted':'aborted'})
                                                                      
            StateMachine.add('GREETING',
                             Greeting(self.__helper_obj),
                             transitions={'complete':'complete'})

    def moveHeadComplete(self, userdata, status, result):
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission two cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Greet mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission two aborted'
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Greet mission failed during head move action'

    def face_recognition_result_cb(self, userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Face recognition action complete
            local_dict = userdata.seen_dict_in
            if len(result.ids_detected) > 0:
                # Recognised faces detected. Have we seen them before or are they new
                for idx, val in enumerate(result.ids_detected):
                    if val not in local_dict:
                        # Add to dictionary
                        local_dict[val] = result.names_detected[idx]

                        # Log who was seen                                     
                        rospy.loginfo("Greeting: I have seen %s", result.names_detected[idx])

                # Update disctionary stored in user data        
                userdata.seen_dict_out = local_dict
                
        elif status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission two cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Greet mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission two aborted'
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Greet mission failed during scan for faces action'

        # By not returning anything the state will return with the corresponding outcome of the action


# PREPARE_FOR_MOVEMENT_GRT State
class PrepareMovementGeeting(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete','move'],
                       input_keys=['start_in'],
                       output_keys=['start_out','seen_dict','user_data_absolute','user_data_pan','user_data_tilt'])
        self.__helper_obj = helper_obj
                                                             
    def execute(self, userdata):
        # Is this the start of a new mission
        if userdata.start_in == True:
            userdata.start_out = False
            # clear the seen dictionary
            userdata.seen_dict = {}
            scan_complete = False
            # get the camera start position (pan min and tilt max)
            position_request_pan,  position_request_tilt = self.__helper_obj.CameraToStartPos()
        else:            
            scan_complete, position_request_pan, position_request_tilt = self.__helper_obj.CameraToNextPos()
        
        # Set up user data that will be used for goal in next state if not complete
        userdata.user_data_absolute = True
        userdata.user_data_pan = position_request_pan
        userdata.user_data_tilt = position_request_tilt        

        if scan_complete == True:
            next_outcome = 'complete'
        else:
            next_outcome = 'move'
        
        return next_outcome 
                                                           
# Greeting State
class Greeting(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete'],
                       input_keys=['seen_dict'], output_keys=['mission_status'])
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
        
        userdata.mission_status = 'Greet mission complete'
        
        return 'complete'

