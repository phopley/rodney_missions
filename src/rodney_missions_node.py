#!/usr/bin/env python
# This ROS node is the state machine for the missions

import sys
import rospy
import missions_lib
from std_msgs.msg import String, Empty
from smach import State, StateMachine
from smach_ros import MonitorState, SimpleActionState, IntrospectionServer
from face_recognition_msgs.msg import scan_for_facesAction, scan_for_facesGoal
                          
# The PREPARE state
class Prepare(State):
    def __init__(self):
        State.__init__(self, outcomes=['mission2','done_task'], input_keys=['mission'])
    
    def execute(self, userdata):        
        # Based on the userdata either change state to the required mission or carry out single task
        # userdata.mission contains the mission or single task and a number of parameters seperated by '-'
        retVal = 'done_task';
        
        # Split into parameters using '-' as the delimiter
        parameters = userdata.mission.split("-")
        
        if parameters[0] == 'M2':
            # Mission 2 is scan for faces and greet those known, there are no other parameters with this mission request
            retVal = 'mission2'
            
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

        
# Top level state machine. The work for each mission is another state machine in the 'mission' states        
class RodneyMissionsNode:

    def __init__(self):
        rospy.on_shutdown(self.ShutdownCallback)
        
        # Subscribe to message to cancel missions        
        self.__cancel_sub = rospy.Subscriber('/missions/mission_cancel', Empty, self.CancelCallback)
        
        # Create top level state machine
        self.__sm = StateMachine(['preempted'])
        with self.__sm:
            # Add the first state which monitors for a mission to run
            StateMachine.add('WAITING',
                             MonitorState('/missions/mission_request',
                             String,
                             self.MissionRequestCB,
                             output_keys = ['mission']),
                             transitions={'valid':'WAITING', 'invalid':'PREPARE', 'preempted':'preempted'}) 
            # Add state to prepare the mission
            StateMachine.add('PREPARE',
                             Prepare(),
                             transitions={'mission2':'MISSION2','done_task':'WAITING'})
            # Add the reporting state
            StateMachine.add('REPORT',
                             Report(),
                             transitions={'success':'WAITING'})
                             
            # Create a sub state machine for mission 2 - greeting
            self.__sm_mission2 = StateMachine(['success', 'aborted', 'preempted'])
            
            with self.__sm_mission2:
                goal_scan = scan_for_facesGoal()                
                StateMachine.add('SCANNING',
                                 SimpleActionState('head_control_node',
                                                   scan_for_facesAction,
                                                   goal=goal_scan,
                                                   result_slots=['detected']),                                 
                                 transitions={'succeeded':'GREETING', 'aborted':'aborted', 'preempted':'preempted'})
                StateMachine.add('GREETING',
                                 missions_lib.Greeting(),                                 
                                 transitions={'success':'success'})
                                 
            # Now add the sub state machine (for mission 2) to the top level one
            StateMachine.add('MISSION2', 
                             self.__sm_mission2, 
                             transitions={'success':'REPORT', 'aborted':'WAITING', 'preempted':'WAITING'}) 
        
        # Create and start the introspective server so that we can use smach_viewer
        sis = IntrospectionServer('server_name', self.__sm, '/SM_ROOT')
        sis.start()
                             
        self.__sm.execute()
        
        # Wait for ctrl-c to stop application
        rospy.spin()
        sis.stop()
        
    
    # Monitor State takes /missions/mission_request topic and passes the mission in user_data to the PREPARE state
    def MissionRequestCB(self, userdata, msg):                
        # Take the message data and send it to the next state in the userdata
        userdata.mission = msg.data;       
                        
        # Returning False means the state transition will follow the invalid line
        return False
        
    # Callback for cancel mission message
    def CancelCallback(self, data):
        # List all sub state machines which can be preempted
        self.__sm_mission2.request_preempt()
        
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
    
