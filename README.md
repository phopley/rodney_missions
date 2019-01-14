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
  
* `speech/to_speak`:  
  Publishes `speech/voice` A message with either text containing the words to speak or wav containing the path of the wav file to play
  
* `missions/acknowledge` Some missions require the operator to acknowledge before continuing
  
* `robot_face/text_out`:  
  Publishes `std_msgs/String` Text for the robot face to animate the lips to
  
* `missions/mission_complete`:  
  Publishes `std_msgs/String` Message indicating that the current mission completed

Action Clients:
* `head_control_node`: To request movement of the head
* `face_recognition`: To request and obtain the result of a face recognition scan on an image from the camera
* `object_detection`: To request and obtain the result of a object detection scan on an image from the camera

Parameters:

* `/servo/index0/pan/min`: minimum range for the head pan servo. Default value = -(pi/2.0)
* `/servo/index0/pan/max`: Maximum range for the head pan servo. Default value = pi/2.0
* `/servo/index0/tilt/min`: Minimum range for the head tilt servo. Default value = -(pi/2.0)
* `/servo/index0/tilt/max`: Maximum range for the head tilt servo. Default value = pi/2.0
* `/head/position/pan`: The pan position that the head will move to when a task is complete (default positon). Default value = 0.0
* `/head/position/tilt`: The tilt position that the head will move to when a task is complete (default positon). Default value = 0.0
* `/head/view_step/pan`: The increment that the pan servo will move for each scan. Default value = 0.436332
* `/head/view_step/tilt`: The increment that the tile servo will move for each scan. Default value = 0.436332
* `/head/manual_view_step/pan`: Pan step value to move the camera in manual mode. Default = 0.174533
* `/head/manual_view_step/tilt`: Tilt step value to move the camera in manual mode. Default = 0.174533

State Machines:

![alt text](https://github.com/phopley/rodney_missions/blob/master/smach.png "State Machine")

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
  
* `MISSION2`:  
  A sub state machine resposible for controlling mission 2, face recognition
  
  * `PREPARE_FOR_MOVEMENT_GRT`:  
    A state which sets up the next required position of the head/camera
    
  * `MOVE_HEAD_GRT`:  
    A SimpleActionState that requests the head/camera be moved
  
  * `SCAN_FOR_FACES`:  
    A SimpleActionState that requests the faces recognition operation be carried out on the current image from the camera
    
  * `GREETING`:  
    A state responsible for forming the response to any faces recognised
    
* `MISSION3`:  
  A sub state machine resposible for controlling mission 3, object recognition
  
  * `PREPARE_FOR_MOVEMENT_OS`:  
    A state which sets up the next required position of the head/camera

  * `MOVE_HEAD_OS`:  
    A SimpleActionState that requests the head/camera be moved
    
  * `SCAN_FOR_OBJECTS`:  
    A SimpleActionState that requests the scan for objects operation be carried out on the current image from the camera
    
  * `CHOICE_OS`:  
    A state which decides on the next state to transit to based on if the object in question was detected or not.
    
  * `WAIT_FOR_USER_OS`:  
     A monitor state for the /missions/acknowledge topic
 

