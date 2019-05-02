# rodney_missions
ROS node to control the different missions conducted by the house bot Rodney. It also controls some of the simple jobs.

## Running the Node

Once you have the node built you can run the rodney robot by launching the rodney.launch file.

## Node Information
Topics:
* `missions/mission_request`:  
  Subscribes using MonitorState `std_msgs/String` A request to carry out a robot mission or simple job
* `missions/mission_cancel`:  
  Subscribes `std_msgs/String` A request to cancel the current robot mission
* `missions/acknowledge`:    
  Subscribes `sts_msgs/Empty` An acknowledgement by a user that the mission can move on
* `speech/to_speak`:  
  Publishes `speech/voice` A message with either text containing the words to speak or wav containing the path of the wav file to play
* `robot_face/text_out`:  
  Publishes `std_msgs/String` Text for the robot face to animate the lips to
* `missions/mission_complete`:  
  Publishes `std_msgs/String` Message indicating that the current mission completed
* `commands/lidar_enable`:  
  Publishes `std_msgs/Bool` Message indicating if the LIDAR should be enabled or not

Action Clients:
* `head_control_node`: To request movement of the head
* `face_recognition`: To request and obtain the result of a face recognition scan on an image from the camera
* `move_base`: To reuest that the base is navigated to a new location

Service Clients:
* `stop_motor`: To disable the LIDAR
* `start_motor`: To enable the LIDAR

Parameters:

* `/servo/index0/pan/min`: minimum range for the head pan servo. Default value = -(pi/2.0)
* `/servo/index0/pan/max`: Maximum range for the head pan servo. Default value = pi/2.0
* `/servo/index0/tilt/min`: Minimum range for the head tilt servo. Default value = -(pi/2.0)
* `/servo/index0/tilt/max`: Maximum range for the head tilt servo. Default value = pi/2.0
* `/head/position/pan`: The pan position that the head will move to when a task is complete (default positon). Default value = 0.0
* `/head/position/tilt`: The tilt position that the head will move to when a task is complete (default positon). Default value = 0.0
* `~head/view_step/pan`: The increment that the pan servo will move for each scan. Default value = 0.436332
* `~head/view_step/tilt`: The increment that the tile servo will move for each scan. Default value = 0.436332
* `~head/manual_view_step/pan`: Pan step value to move the camera in manual mode. Default = 0.174533
* `~head/manual_view_step/tilt`: Tilt step value to move the camera in manual mode. Default = 0.174533
* `~head/user_position/pan`: Pan position that the head will move to for user input. Default value = 0.0
* `~head/user_position/tilt`: Tilt position that the head will move to for user input. Default value = -0.5

State Machines:

![alt text](https://github.com/phopley/rodney_missions/blob/master/smach.png "Root State Machine")

The top level state machine contains the following states
* `WAITING`:  
  Is a monitor state for the /missions/mission_request topic
* `PREPARE`:  
  A state that examines a request and either actions a job or hands control to a mission sub state machine
* `REPORT`:  
  A state that reports that a mission is complete
* `DEFAULT_HEAD_POSITION`:  
  A SimpleActionState that returns the head/camera to the default position
* `MOVE_HEAD`:  
  A SimpleActionState that request sthe head/camera be moved

![alt text](https://github.com/phopley/rodney_missions/blob/master/smach_mission1.png "Mission 1 State Machine")

* `MISSION1`:  
  A sub state machine responsible for controlling mission 1, "take a message to.."
  * `PREPARE_MISSION`:  
    A state that loads the given waypoint file
  * `PREPARE_FOR_HEAD_MOVE`:  
    A state which sets up the next required position of the head/camera
  * `MOVE_HEAD`:  
    A SimpleActionState that requests the head/camera be moved 
  * `SCAN_FOR_FACES`:  
    A SimpleActionState that requests the faces recognition operation be carried out on the current image from the camera   
  * `CHECK_FOR_SUBJECT`:  
    A state that checks subjects seen by the camera to the ID of the subject we are searching for. If the subject is seen the message is delivered
  * `WAIT_FOR_USER`:  
    State that waits for the acknowledge message or timesout after 60 seconds
  * `DEFAULT_HEAD_POISTION`:  
    A SimpleActionState that returns the head/camera to the default position
  * `PREPARE_TO_MOVE`:  
    A state machine that constructs a nav goal for the MOVE state
  * `MOVE`:  
    A SimpleActionState that moves the robot base to the given nav goal

![alt text](https://github.com/phopley/rodney_missions/blob/master/smach_mission2.png "Mission 2 State Machine")

* `MISSION2`:  
  A sub state machine responsible for controlling mission 2, "greet who you recognise"
  * `PREPARE_FOR_MOVEMENT_GRT`:  
    A state which sets up the next required position of the head/camera
  * `MOVE_HEAD_GRT`:  
    A SimpleActionState that requests the head/camera be moved
  * `SCAN_FOR_FACES`:  
    A SimpleActionState that requests the faces recognition operation be carried out on the current image from the camera
  * `GREETING`:  
    A state responsible for forming the response to any faces recognised

![alt text](https://github.com/phopley/rodney_missions/blob/master/smach_mission4.png "Mission 4 State Machine")

* `MISSION4`:
  A sub state machine responsible for controlling mission 4, "go home"
  * `PREPARE_MISSION`:  
    A state that loads a given waypoint file and extracts the home location and position
  * `DEFAULT_HEAD_POSITION`:  
    A SimpleActionState that returns the head/camera to the default position
  * `MOVE`:  
    A SimpleActionState that moves the robot base to the given nav goal (home position)

## License
Software source and object files are licensed under the Apache License, Version 2.0. See the License for the specific language governing permissions and limitations under the License.
