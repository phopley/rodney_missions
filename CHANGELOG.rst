^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rodney_missions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019/05/02)
------------------
* Some of the parameters on the parameter server are now private
* Added user input position for screen
* Added Mission 1 "Take a message to ..."
* added Mission 4 "Go Home"

1.1.0 (2019/04/10)
------------------
* Added functionality to enable/disable LIDAR

1.0.1 (2019/01/14)
------------------
* When manual head movement will exceed min/max instead of not moving the head will move to the min/max

1.0.0 (2019/01/12)
------------------
* Head movement now controlled by an action state
* Changed to use radians and the standard ROS orientation
* Changed to use action state to start face recognition
* Sub state machines moved into libraries

0.2.0 (2018-12-05)
------------------
* Added mission 3
* Mission 2 no longer uses action messages

0.1.0 (2018-11-12)
------------------
* First formal release of the package
