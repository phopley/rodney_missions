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
# Statemachine that runs mission 4. Return to home location
import rospy
import yaml
import io
from smach import State, StateMachine
from smach_ros import SimpleActionState
from head_control.msg import point_headAction, point_headGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Child (derived) class. Parent class is StateMachine
class Mission4StateMachine(StateMachine):
    def __init__(self, helper_obj):
        StateMachine.__init__(self, outcomes=['complete','preempted','aborted'], input_keys=['mission_data'], output_keys=['mission_status'])
        
        self.__helper_obj = helper_obj
        
        with self:
            # This state will prepare for the mission
            StateMachine.add('PREPARE_MISSION',
                             PrepareMission4(self.__helper_obj),
                             transitions={'ready':'DEFAULT_HEAD_POSITION','error':'complete'})
            
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
                             transitions={'succeeded':'MOVE','preempted':'preempted','aborted':'aborted'})                             
                                                    
            # This state uses an Action to move the robot to the required goal
            StateMachine.add('MOVE',
                             SimpleActionState('move_base',
                                               MoveBaseAction,
                                               goal_slots=['target_pose'],
                                               result_cb = self.moveComplete,
                                               output_keys=['mission_status']),
                             transitions={'succeeded':'complete', 'preempted':'preempted', 'aborted':'aborted'},
                             remapping={'target_pose':'destination'})                             
                             
                             
    def defaultHeadPositionComplete(self, userdata, status, result):
    
        self.__helper_obj.CameraAtDefaultPos(userdata, status, result)
        
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission number 4 cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Go to waypont mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission 4 aborted'                   
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Go to waypont mission failed during move head action'            
                                     
    def moveComplete(self, userdata, status, result):
        if status == GoalStatus.PREEMPTED:
            # Report with voice that the mission was cancelled
            message = 'Mission number 4 cancelled'
            self.__helper_obj.Speak(message, message)
            
            # set the mission status
            userdata.mission_status = 'Go to waypont mission cancelled'
            
        elif status == GoalStatus.ABORTED:
            # Report with voice that the mission was aborted
            message = 'Warning mission 4 aborted'
            self.__helper_obj.Speak(message, message + ':(')
            
            # set the mission status
            userdata.mission_status = 'Go to waypont mission failed during base move action'
            
        elif status == GoalStatus.SUCCEEDED:
            userdata.mission_status = 'Go to waypont mission complete'
            

# PREPARE_MISSION State. Prepares the mission by loading the waypoint file
class PrepareMission4(State):   
    def __init__(self, helper_obj):
        State.__init__(self, outcomes=['ready','error'],
                       input_keys=['mission_data'],
                       output_keys=['destination','mission_status'])
        self.__helper_obj = helper_obj
        
    def execute(self, userdata):
        # Parse the mission data using '|' as the delimiter        
        # parameters[0] will contain the filename of the file containing the poses
        # parameters[1] if it exists will contain the way point to move to. If not waypoint given then go home
        parameters = userdata.mission_data.split("|")
                
        
        # Ensure the Lidar is enabled
        self.__helper_obj.LidarEnable()
        
        # Load the waypoints
        try:
            with open(parameters[0], 'r') as stream:
                try:                    
                    waypoints = yaml.load(stream)
                    next_outcome = 'ready'
                except:
                    rospy.logerr('Bad waypoint file')
                    userdata.mission_status = 'Bad waypoint file'
                    next_outcome = 'error'
        except:
            rospy.logerr("Can't open waypoint file")
            userdata.mission_status = "Can't open waypoint file"
            next_outcome = 'error'                    
        
        if next_outcome == 'ready':        
            # Was a waypoint include in the mission_data or are we going home
            if len(parameters) > 1:
                target_waypoint = parameters[1]
            else:
                target_waypoint = 'home'
                
            # Check the waypoint exists
            if target_waypoint in waypoints:            
                # Copy the waypoint data in to a PoseStamped message and userdata                            
                waypoint = waypoints[target_waypoint]
        
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
 
            else:
                rospy.logerr('Given waypoint not in file')
                userdata.mission_status = 'Given waypoint not in file'
                next_outcome = 'error'                  
    
        return next_outcome                                    

