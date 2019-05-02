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
# Statemachine that runs mission 1. Searches for a known person and
# delivers a message
import rospy
import yaml
import io
import threading
from smach import State, StateMachine
from smach_ros import SimpleActionState
from head_control.msg import point_headAction, point_headGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from face_recognition_msgs.msg import scan_for_facesAction
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatus

# Child (derived) class. Parent class is StateMachine
class Mission1StateMachine(StateMachine):
    def __init__(self, helper_obj):
        StateMachine.__init__(self, outcomes=['complete','preempted','aborted'], input_keys=['mission_data'], output_keys=['mission_status'])

        self.__helper_obj = helper_obj
        
        with self:
            # This state will prepare for the mission
            StateMachine.add('PREPARE_MISSION',
                             PrepareMission1(self.__helper_obj),
                             transitions={'ready':'PREPARE_FOR_HEAD_MOVE','error':'aborted'})
            
            # Set up action goal for deafult head position            
            default_position_pan, default_position_tilt = self.__helper_obj.CameraDefaultPos()
            head_goal = point_headGoal()
            head_goal.absolute = True
            head_goal.pan = default_position_pan
            head_goal.tilt = default_position_tilt
                             
            # Add the default camera position state. Which moves the head to the default position
            StateMachine.add('DEFAULT_HEAD_POSITION',
                             SimpleActionState('head_control_node',
                                               point_headAction,
                                               goal = head_goal,
                                               result_cb = self.defaultHeadPositionComplete,
                                               output_keys=['mission_status']),                                               
                             transitions={'succeeded':'PREPARE_TO_MOVE','preempted':'preempted','aborted':'aborted'}) 
                             
            # The next state prepares for each nav goal request
            StateMachine.add('PREPARE_TO_MOVE',
                             PrepareToMove(),
                             transitions={'done':'MOVE'},
                             remapping={'current_waypoint_in':'current_waypoint','current_waypoint_out':'current_waypoint',
                                        'waypoint_direction_in':'waypoint_direction','waypoint_direction_out':'waypoint_direction'})
                                        
            # This state uses an Action to move the robot to the required goal
            StateMachine.add('MOVE',
                             SimpleActionState('move_base',
                                               MoveBaseAction,
                                               goal_slots=['target_pose'],
                                               result_cb = self.moveComplete,
                                               output_keys=['mission_status']),
                             transitions={'succeeded':'PREPARE_FOR_HEAD_MOVE', 'preempted':'preempted', 'aborted':'aborted'},
                             remapping={'target_pose':'destination'})
                             
            # This state will calculate the next head/camera position
            StateMachine.add('PREPARE_FOR_HEAD_MOVE',
                             PrepareToMoveHead(self.__helper_obj),
                             transitions={'scan_complete':'DEFAULT_HEAD_POSITION','move_head':'MOVE_HEAD'},
                             remapping={'start_in':'start','start_out':'start'})
                             
            # This state will call the action to move the head/camera
            StateMachine.add('MOVE_HEAD',
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
                                               result_slots=['ids_detected','names_detected'],
                                               result_cb = self.scanForFaceComplete,
                                               output_keys=['mission_status']),                                                                                                  
                             transitions={'succeeded':'CHECK_FOR_SUBJECT','preempted':'preempted','aborted':'aborted'},
                             remapping={'ids_detected':'seen_ids','names_detected':'seen_names'})
                             
            # This state will check the results of the scan and decide on the next action
            StateMachine.add('CHECK_FOR_SUBJECT',
                             CheckForSubject(self.__helper_obj),
                             transitions={'seen':'WAIT_FOR_USER','not_seen':'PREPARE_FOR_HEAD_MOVE'})
                             
            # This state will wait for the acknowledge message or will timeout if the message is not received in a set time
            StateMachine.add('WAIT_FOR_USER',
                             WaitForMsg(self.__helper_obj),
                             transitions={'ack':'complete','timeout':'SCAN_FOR_FACES','preempted':'preempted'})
            
                                                          
    def defaultHeadPositionComplete(self, userdata, status, result):
    
        self.__helper_obj.CameraAtDefaultPos(userdata, status, result)
        
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission number 1 cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Take a message mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission number 1 aborted'                   
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Take a message mission failed during move head action'
            
    def moveComplete(self, userdata, status, result):
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission number 1 cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Take a message mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission number 1 aborted'
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Take a message mission failed during base move action'            
            
    def moveHeadComplete(self, userdata, status, result):
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission number 1 cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Take a message mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission one aborted'
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Take a message mission failed during head move'
            
    def scanForFaceComplete(self, userdata, status, result):
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission number 1 cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Take a message mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission one aborted'
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Take a message mission failed during face scan'        
        
        
# WAIT_FOR_USER state. Waits for the ack message for a set time
class WaitForMsg(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['ack','timeout','preempted'],
                       output_keys=['mission_status'])
        
        self.__helper_obj = helper_obj
                               
        self.__mutex = threading.Lock()
        self.__msg = None
        self.__subscriber = rospy.Subscriber('/missions/acknowledge', Empty, self.callBack, queue_size=1)
        
    def callBack(self, msg):
        # Indicate a message was received
        self.__mutex.acquire()
        self.__msg = msg
        self.__mutex.release()
        
    def execute(self, userdata):
        # Clear any previous ack message
        self.__mutex.acquire()
        self.__msg = None
        self.__mutex.release()
    
        # Give the user 60 seconds to acknowledge the message
        timeout = rospy.Time.now() + rospy.Duration.from_sec(60.0)
        message_arrived = False
        preempted = False
        while rospy.Time.now() < timeout and message_arrived == False and preempted == False:
            # Check to see if message arrived            
            self.__mutex.acquire()
            if self.__msg is not None:
                # Message has arrived
                message_arrived = True
            self.__mutex.release()
            
            # Check mission was cancelled
            if self.preempt_requested():
                self.service.preempt()
                preempted = True
             
        # Why did the loop terminate   
        if preempted == True:
            # Report with voice that the mission was cancelled
            message = 'Mission one cancelled'
            self.__helper_obj.Speak(message, message)            
            # set the mission status
            userdata.mission_status = 'Take a message mission cancelled'
            next_outcome = 'preempted'
        elif message_arrived == True:
            userdata.mission_status = 'Take a message mission complete'
            next_outcome = 'ack'            
        else:
            next_outcome = 'timeout'
            
        return next_outcome
                                                               
                      
# CHECK_FOR_SUBJECT state. Checks the results from the previous state to see if person searching for was found
class CheckForSubject(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['seen','not_seen'],
                       input_keys=['person_id','message','seen_ids','seen_names'])
        self.__helper_obj = helper_obj

    def execute(self, userdata):
        next_outcome = 'not_seen' 
                
        # Was anyone see?  
        if len(userdata.seen_ids) > 0:                     
            # at least one person was seen
            for idx, val in enumerate(userdata.seen_ids):               
                if str(val) == userdata.person_id:                    
                    # We have found who we were looking for
                    next_outcome = 'seen'                                                    
                    rospy.loginfo("I have found %s, delivering message", userdata.seen_names[idx])
                    greeting = 'Hello ' + userdata.seen_names[idx] + ' I have a message for you'
                    
                    # Speak greeting
                    self.__helper_obj.Speak(greeting, greeting)
                    
                    rospy.sleep(2.0)
                    
                    # Speak message
                    self.__helper_obj.Speak(userdata.message, userdata.message)                                
            
        return next_outcome                      


# PREPARE_FOR_HEAD_MOVE State. Prepares for the next head position
class PrepareToMoveHead(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['scan_complete','move_head'],
                       input_keys=['start_in'],
                       output_keys=['start_out','user_data_absolute','user_data_pan','user_data_tilt'])
        self.__helper_obj = helper_obj
                                                             
    def execute(self, userdata):
        # Is this the start of a new head scan
        if userdata.start_in == True:
            userdata.start_out = False
            scan_complete = False
            # get the camera start position (pan min and tilt max)
            position_request_pan,  position_request_tilt = self.__helper_obj.CameraToStartPos()
        else:            
            scan_complete, position_request_pan, position_request_tilt = self.__helper_obj.CameraToNextPos()
        
        # Set up user data that will be used for goal in next state if scan not complete
        userdata.user_data_absolute = True
        userdata.user_data_pan = position_request_pan
        userdata.user_data_tilt = position_request_tilt        

        if scan_complete == True:
            next_outcome = 'scan_complete'
        else:
            next_outcome = 'move_head'
        
        return next_outcome 

                             
# PREPARE_TO_MOVE State. Reads the next waypoint to move the base to
class PrepareToMove(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'],
                       input_keys=['waypoints','current_waypoint_in','waypoint_direction_in'],
                       output_keys=['current_waypoint_out','destination','waypoint_direction_out','start']) 
                       
    def execute(self, userdata):
        # Extract the next waypoint but which way along the waypoints are we travelling
        if userdata.waypoint_direction_in == True:
            # Incrementing along the waypoints
            next_waypoint = userdata.current_waypoint_in + 1
            waypoint_id = 'w' + str(next_waypoint)
                    
            # If next waypoint does not exist (reached end of list) we need to work backwards along the list
            if not waypoint_id in userdata.waypoints:
                next_waypoint = userdata.current_waypoint_in - 1
                userdata.waypoint_direction_out = False
                # Allow for only one waypoint
                if next_waypoint == 0:
                    next_waypoint = 1
                    
                # next waypoint updated so update the waypoint_id string
                waypoint_id = 'w' + str(next_waypoint)
        else:
            # Decrementing along the waypoints
            next_waypoint = userdata.current_waypoint_in - 1
            
            # If next point is now zero we have reach start of list and we need to work forwards along the list
            if next_waypoint == 0:
                next_waypoint = 1
                userdata.waypoint_direction_out = True
                
            waypoint_id = 'w' + str(next_waypoint)
                              
        # Copy the waypoint data in to a PoseStamped message and userdata                            
        waypoint = userdata.waypoints[waypoint_id]
        
        target_pose = PoseStamped()
            
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = waypoint['position']['x']
        target_pose.pose.position.y = waypoint['position']['y']
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = waypoint['orientation']['x']
        target_pose.pose.orientation.y = waypoint['orientation']['y']
        target_pose.pose.orientation.z = waypoint['orientation']['z']
        target_pose.pose.orientation.w = waypoint['orientation']['w']
            
        userdata.destination = target_pose
        userdata.current_waypoint_out = next_waypoint
        
        # Once we reach the new destination we will start a new head scan so reset flag here
        userdata.start = True
        
        return 'done'
                             
# PREPARE_MISSION State. Prepares the mission by loading the waypoint file
class PrepareMission1(State):   
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['ready','error'],
                       input_keys=['mission_data'],
                       output_keys=['person_id','message','waypoints','current_waypoint','waypoint_direction','mission_status','start'])
        self.__helper_obj = helper_obj
        
    def execute(self, userdata):
        # Parse the mission data using '|' as the delimiter        
        # parameters[0] will contain the filename of the file containing the poses
        # parameters[1] will contain the id of the person to search for
        # parameters[2] will contain the message to deliver
        parameters = userdata.mission_data.split("|")
        
        userdata.person_id = parameters[1]
        userdata.message = parameters[2]
        
        # Load patrol poses from the given patrol file
        try:
            with open(parameters[0], 'r') as stream:
                try:                    
                    userdata.waypoints = yaml.load(stream)
                    userdata.current_waypoint = 0
                    userdata.waypoint_direction = True                    
                    userdata.mission_status = 'void' # This will be overwritten
                    next_outcome = 'ready'
                except:
                    rospy.logerr('Bad waypoint file')
                    userdata.mission_status = 'Bad waypoint file'
                    next_outcome = 'error'
        except:
            rospy.logerr("Can't open waypoint file")
            userdata.mission_status = "Can't open waypoint file"
            next_outcome = 'error'                    
        
        # Ensure the Lidar is enabled
        self.__helper_obj.LidarEnable()
        
        # Indicate that when we start scanning for faces that it is a new scan
        userdata.start = True   
    
        return next_outcome                                    
        
