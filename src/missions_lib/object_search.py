#!/usr/bin/env python
# States for searching for an object

import rospy
from std_msgs.msg import Empty
from smach import State, StateMachine
from smach_ros import SimpleActionState, MonitorState
from actionlib_msgs.msg import GoalStatus
from head_control.msg import point_headAction, point_headGoal
from tf_object_detection.msg import scan_for_objectsAction

# Child (derived) class. Parent class is StateMachine
class Mission3StateMachine(StateMachine):
    def __init__(self, helper_obj):
        StateMachine.__init__(self, outcomes=['complete', 'preempted','aborted'], input_keys=['mission_data','start'])

        self.__helper_obj = helper_obj

        with self:
            # This state will calculate the next head/camera position
            StateMachine.add('PREPARE_FOR_MOVEMENT_OS',
                             PrepareMovementObjectSearch(self.__helper_obj),
                             transitions={'complete':'complete','move':'MOVE_HEAD_OS'},
                             remapping={'start_in':'start','start_out':'start'})
                                 
            # This state will call the action to move the head/camera
            StateMachine.add('MOVE_HEAD_OS',
                             SimpleActionState('head_control_node',
                             point_headAction,                                                            
                             goal_slots=['absolute','pan','tilt']),
                             transitions={'succeeded':'SCAN_FOR_OBJECTS','preempted':'preempted','aborted':'aborted'},
                             remapping={'absolute':'user_data_absolute','pan':'user_data_pan','tilt':'user_data_tilt'}) 
                
            # This state will call the action to scan for objects on the image from the camera.
            StateMachine.add('SCAN_FOR_OBJECTS',
                             SimpleActionState('object_detection',
                             scan_for_objectsAction,
                             result_cb=self.object_detection_result_cb,
                             input_keys = ['mission_data'],
                             output_keys = ['object_detected']),
                             transitions={'succeeded':'CHOICE_OS','preempted':'preempted','aborted':'aborted'})                
                
            # This state will will make the decision which state to move to
            # next. If the object in question was detected then it will move
            # to a state that waits for user interaction. If the object was
            # not detected then it will move to the state to move the camera.
            # This seems a bit of an overkill but the SimpleActionState before it
            # can only provide the three possible outcomes 'succeeded', 'aborted'
            # or 'preempted'. This state will request speech to state the
            # object was found.
            StateMachine.add('CHOICE_OS',
                              ChoiceObjectSearch(self.__helper_obj),           
                              transitions= {'move_camera':'PREPARE_FOR_MOVEMENT_OS',
                                            'await_user':'WAIT_FOR_USER_OS'})

            # This state will wait for a message indicating the user wishes
            # to move on. It will allow the user to examine the camera feed
            # once an object was detected
            StateMachine.add('WAIT_FOR_USER_OS',
                             MonitorState('/missions/acknowledge',
                             Empty,
                             self.UserAckOS),
                             transitions={'valid':'WAIT_FOR_USER_OS',
                                          'invalid':'PREPARE_FOR_MOVEMENT_OS',
                                          'preempted':'preempted'})

    def object_detection_result_cb(self, userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            # Extract the result results
            if len(result.names_detected) == 0:
                # No objects recognised
                userdata.object_detected = False
            else:
                # Objects detected, but did we find the one we are interested in?
                userdata.object_detected = False
                for n in result.names_detected:
                    if n == userdata.mission_data:
                        userdata.object_detected = True                      
                        break

        # By not returning anything here (at this indentation) the state will 
        # return with the corresponding outcome of the action 

    def UserAckOS(self, userdata, msg):
        # User acknowledged so move on        
        
        # Returning False means the state transition will follow the invalid outcome
        return False

# PREPARE_FOR_MOVEMENT_OS State
class PrepareMovementObjectSearch(State):
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['complete','move'],
                       input_keys=['start_in'],
                       output_keys=['start_out','user_data_absolute','user_data_pan','user_data_tilt'])
        self.__helper_obj = helper_obj                                 
                       
    def execute(self, userdata):
        # Is this a new search?
        if userdata.start_in == True:
            userdata.start_out = False
            scan_complete = False
            # get the camera start position (pan min and tilt min)
            position_request_pan, position_request_tilt = self.__helper_obj.CameraToStartPos()
        else:            
            scan_complete, position_request_pan, position_request_tilt = self.__helper_obj.CameraToNextPos()

        # Set up user data that will be used for goal in next state if not complete
        userdata.user_data_absolute = True
        userdata.user_data_pan = position_request_pan
        userdata.user_data_tilt = position_request_tilt

        if scan_complete == True:
            greeting = "Mission complete Rodney standing by"            
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

