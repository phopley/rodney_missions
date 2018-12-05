# rodney_missions
ROS node to control the different missions conducted by the house bot Rodney. It also controls some of the simple jobs.

## Running the Node

Once you have the node built you can run the rodney robot by launching the rodney.launch file.

## Node Information
Topics:

* `missions/mission_request`:  
  Subscribes `std_msgs/String` A request to carry out a robot mission or simple job
  
* `missions/mission_cancel`:  
  Subscribes `std_msgs/String` A request to cancel the current robot mission
  
* `/head_control_node/head_move_complete`:  
  Subsribes `std_msgs/Empty` An indication that the last request to move the head/camera was completed
  
* `/face_recognition_node/result`:  
  Subscribes `face_recognition_msgs/face_recognition` Result from the last face recognition scan request

* `/tf_object_detection_node/result`:  
  Subscribes `tf_object_detection/detection_results` Result from the last object detection scan request
  
* `/missions/acknowledge`:  
  Subsribes `std_msgs/Empty` Some missions require operator acknowledge before continuing

* `speech/to_speak`:  
  Publishes `speech/voice` A message with either text containing the words to speak or wav containing the path of the wav file to play  
  
* `robot_face/text_out`:  
  Publishes `std_msgs/String` Text for the robot face to animate the lips to
  
* `head_control_node/head_position_request`:  
  Publishes `servo_msgs/pan_tilt` A request the movement of the head/camera
  
* `missions/mission_complete`:  
  Publishes `std_msgs/String` Message indicating that the current mission completed
  
* `face_recognition_node/start`:  
  Publishes `std_msgs/Empty` A request to scan the next camera image for face recognition
  
* `tf_object_detection_node/start`:  
  Publishes `std_msgs/Empty` A request to scan the next camera image for object detection

Parameters:

* `/servo/index0/pan_min`: minimum range for the head pan servo. Default value = 0.
* `/servo/index0/pan_max`: Maximum range for the head pan servo. Default value = 180.
* `/servo/index0/tilt_min`: Minimum range for the head tilt servo. Default value = 0.
* `/servo/index0/tilt_max`: Maximum range for the head tilt servo. Default value = 180.
* `/head/position/pan`: The pan position that the head will move to when a task is complete (default positon). Default value = 90.
* `/head/position/tilt`: The tilt position that the head will move to when a task is complete (default positon). Default value = 45.
* `/head/view_step/pan`: The increment that the pan servo will move for each scan. Default value = 10.
* `/head/view_step/tilt`: The increment that the tile servo will move for each scan. Default value = 10.

State Machines:

![alt text](https://github.com/phopley/rodney_missions/blob/mission3/smach.png "State Machine")

The top level state machine contains the following states
* `WAITING`:  
  Is a monitor state for the /missions/mission_request topic
  
* `PREPARE`:  
  A state that examines a reuqest and either actions a job or hands control to a mission state

* `REPORT`:  
  A state that reports that a mission is complete
  
* `SUB_PREEMPTED`:  
  A state that returns the head/camera to the default position
  
* `MISSION2`:  
  A sub state machine resposible for controlling mission 2
  
  * `MOVE_CAMERA_GRT`:  
    A state which request that the head/camera be moved to the next position
    
  * `WAITING_FOR_MOVEMENT_GRT`:  
    A monitor state for the /head_control_node/head_move_complete topic and then request a face recognition scan
  
  * `WAITING_FOR_ANALYSIS_GRT`:  
    A monitor state for the /face_recognition_node/result topic
    
  * `GREETING`:  
    A state reposible for forming the response to recognisation someone or if no one is recognised

* `MISSION3`:  
  A sub state machine resposible for controlling mission 3
  
  * `MOVE_CAMERA_OS`:  
    A state which request that the head/camera be moved to the next position
    
  * `WAITING_FOR_MOVEMENT_OS`:  
    A monitor state for the /head_control_node/head_move_complete topic and then request an object detection scan
    
  * `WAITING_FOR_ANALYSIS_OS`:  
    A monitor state for the /tf_object_detection_node/result topic
    
  * `CHOICE_OS`:  
    This state makes a choice of the next state based on the result of the object detection scan
    
  * `WAIT_FOR_USER_OS`:  
    A monitor state for the /missions/acknowledge topic
