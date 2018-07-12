#!/usr/bin/env python
# This ROS node is the action server for missions. It will run a state machine based on the mission selected.

import sys
import rospy
import actionlib
from rodney_missions.msg import missionAction, missionGoal, missionResult 

class RodneyMissionsNode:

    def __init__(self):
        self.__server = actionlib.SimpleActionServer('missions', missionAction, self.DoMission, False)
        self.__server.start()

    # Callback to start a goal (mission)
    def DoMission(self, goal):
        # TODO for now we will publish the result straight away
        result = missionResult()
        self.__server.set_succeeded(result)        
        
def main(args):
    rospy.init_node('rodney_missions_node', anonymous=False)
    rmn = RodneyMissionsNode()
    rospy.loginfo("Rodney missions node started")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
