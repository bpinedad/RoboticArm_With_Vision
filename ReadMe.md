## Dependencies:
* Mediapipe - pip3 install mediapipe
* OpenCV - pip3 install opencv-python

## ROS Instructions:
* Tested with Noetic. Python 3 was used, may present issues with other ROS releases.
* Rosserial required. For noetic build from source. https://github.com/ros-drivers/rosserial
* Run master node
  * roscore
* Run Rosserial
  * sudo chmod a+rw /dev/ttyUSB0
  * rosrun rosserial_python serial_node.py /dev/ttyUSB0 
* Run from launch file
*  roslaunch roboticArm_With_Vision robotVision.launch

## References:
* Hand Detection: https://www.analyticsvidhya.com/blog/2021/07/building-a-hand-tracking-system-using-opencv/
* Robot Build and Joystick code: https://wiki.keyestudio.com/Ks0198_keyestudio_4DOF_Robot_Mechanical_Arm_Kit_for_Arduino_DIY

## Summary
Base code is the one found in reference link. From that, hand positions are calculated using the relation between points from right hand using the following diagram:
* ![image](https://user-images.githubusercontent.com/49768807/136865461-2755365b-af49-41dc-b3eb-3dba6ff1eb7e.png)

## Movement
* **Servo 1 (Left - Right):** Rotates depending on position of hand using as reference Palm center **"Point 9"**. Determines left or right position according to where in the frame the point is in X direction.
* **Servo 2 (Up - Down):** Moves depending on position of hand using as reference Palm center **"Point 9"**. Determines up or down position according to where in the frame the point is in Y direction.
* **Servo 3 (Reach):** Moves depending on distance between Wrist **"Point 0"** and **"Point 1"**. Determines stretch or contract according to how far is the hand form the camera.
* **Servo 4 (Open - Close):** Moves depending on distance between Wrist **"Point 0"** and Middle finger tip **"Point 12"**. Determines close or open position for gripper mirroring hand being open or closed.
