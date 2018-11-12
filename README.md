# rodney_missions
ROS node to control the different missions conducted by the house bot Rodney.
It also controls some of the simple jobs.

## Running the Node

Once you have the node built you can run the rodney robot by launching the rodney.launch file.

## Node Information
Topics:

* `missions/mission_request`:
  Subscribes `std_msgs/String` A request to carry out a robot mission or simple job

* `missions/mission_cancel`:
  Subscribes `std_msgs/String` A request to cancel the current robot mission
  
* `speech/to_speak`:
  Publishes `speech/voice` A message with either text containing the words to speak or wav containing the path of the wav file to play
  
* `robot_face/text_out`:
  Publishes `std_msgs/String` Text for the robot face to animate the lips to
  
* `head_control_node/manual`:
  Publishes `std_msgs/String` A request the movement of the camera/head
  
* `missions/mission_complete`:
  Publishes `std_msgs/String` Message indicating that the current mission completed
  
Actions:

* `face_recognition_msgs/scan_for_faces`:  
  Client used to request the  head/camera is moved and scanning for a recognised face at each set position is conducted
  
State Machines:

![alt text](https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-statemachine1.png "State Machine")

The top level state machine contains the following states
* `WAITING`:
  Is a monitor state for the /missions/mission_request topic
  
* `PREPARE`:
  A state that examines a reuqest and either actions a job or hands control to a mission state
  
* `REPORT`:
  A state that reports that a mission is complete
  
* `MISSION2`:
  A sub state machine resposible for controlling mission 2
  
  * `SCANNING`:
    A simple action state responsible for requesting the `face_recognition_msgs/scan_for_faces` action
    
  * `GREETING`:
    A state reposible for forming the response to recognisation someone or if no one is recognised
