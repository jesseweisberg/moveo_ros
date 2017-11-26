# moveo_ros
ROS packages that can be used to plan and execute motion trajectories for the BCN3D Moveo robotic arm in simulation and real-life.  

- [Video Demo Here!](https://youtu.be/2RcTTqs17O8)

## How to Use:

### Getting the BCN3D Simulation Working with Motion Planning
![moveit_screenshot.png](/moveit_screenshot.png)

1. Make sure you have ROS installed correctly with a functioning workspace-- I used ROS Kinetic on Ubuntu 16.04 (if you have a different distro, you may need to change some things).  I currently have 'moveo_ros' in the 'src' folder of my catkin workspace.

2. To plan and execute trajectories for the Moveo in simulation (RVIZ with Moveit plugin), execute the following terminal command:
	```
	roslaunch moveo_moveit_config demo.launch
	```

3. Once the window loads, in the bottom-left corner check "Allow Approximate IK Solutions."  Then click on the "Planning" tab in the MotionPlanning panel of RVIZ.  Select a new goal state by either dragging the interactive marker (light blue ball on the end effector) or under "Select Goal State."  Once goal state is updated, "Plan and Execute" will plan and execute the trajectory from the start state to the updated goal state.


### Moving the real robot, synced with the simulated robot's trajectories.
4. Make sure you download the AccelStepper ([AccelStepper Library Download](http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.57.zip)) and ros_lib ([rosserial-arduino tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)) libraries into your Arduino environment 

5. Change the pin layout between your robot and the RAMPS 1.4 in **'moveo_moveit_arduino.ino'** and upload the file to your Arduino (I'm using MEGA 2560).  Make sure the robot and the simulation are in the same position (to set the simulation upright initially-- select "Upright" from "Select Goal States" in RVIZ.

6. In 'moveit_convert.cpp' replace the stepsPerRevolution array with the steps/revolution (or microsteps/revolution) of each of your motors.  (Note: if you don't already know these values, you can experimentally get how many microsteps/revolution your motors have using the MultiStepperTest.ino and recording/eyeballing the results)

7. With the simulation already running, execute each of the following commands in it's own, separate terminal: 
	- ``` rosrun rosserial_python serial_node.py /dev/ttyUSB0 ```(establishes rosserial node that communicates with Arduino)
	- ```rosrun moveo_moveit moveit_convert ``` (converts simulation joint_state rotations to steps and publishes on the /joint_steps topic, which the Arduino script subscribes to)
	- ```rostopic pub gripper_angle std_msgs/UInt16 <angle 0-180> ```(publishes gripper_angle)

**Now, whatever trajectories are planned and executed in simulation are echoed on the real robot.**

## About Directories
### moveo_urdf
Contains the URDF (Unified Robot Description File) for the BCN3D Moveo. Necessary for simulation in RVIZ and moveit configuration.

### moveo_moveit_config
Configuration for moveit, a motion planning framework that has a plugin in RVIZ, which is what we are using here.

### moveo_moveit
- _moveit_convert.cpp_: Converts simulation 'joint\_state' rotations (from the 'move\_group/fake\_controller\_joint\_states' topic) to steps and publishes on the /joint\_steps topic.  Joint\_steps is an array of 6 Int16 values (though we only have 5 joints in this case) that represent the accumulated steps executed by each joint since the moveit\_convert node has started running. 

- _move\_group\_interface\_coor\_1.cpp_: Can hardcode a pose/position for the end effector in the script and plan/execute a trajectory there.  Also reads/outputs the current pose/position of the end effector.

## Troubleshooting
- After step 7, there should be 3 new topics created: 
	- **/joint\_steps**: steps necessary to move each motor to desired position
	- **/joint\_steps\_feedback**: same as /joint_steps, except published back by arduino to check that information is being received by Arduino correctly 
	- **/gripper\_angle**: current angle of the gripper
