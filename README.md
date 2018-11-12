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
  Publishes `MESSAGE TYPE` WHAT'S IT FOR
  
* `robot_face/text_out`:
  Publishes `MESSAGE TYPE` WHAT'S IT FOR
  
* `head_control_node/manual`:
  Publishes `MESSAGE TYPE` WHAT'S IT FOR
  
* `missions/mission_complete`:
  Publishes `std_msgs/String` Message indicating that the current mission completed
  
Actions:

* `face_recognition_msgs/scan_for_faces`:  
  Client used to request the  head/camera is moved and scanning for a recognised face at each set position is conducted
  
State Machines:

![alt text](https://github.com/phopley/rodney-project/blob/master/docs/images/Opti-statemachine1.png "State Machine")
