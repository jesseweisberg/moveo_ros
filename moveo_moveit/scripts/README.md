# Object-Specific Pick and Place
This script uses real-time object recognition in a monocular image to perform predefined 'pick and place' movements.  In this example, apples are moved to the left, and bananas to the right (though many object-specific grasps and trajectories can be made).

- [Video Demo](https://youtu.be/kkUbyFa2MWc)

## How to Use
1. Connect your webcam via USB to your laptop, with the other USB port connected to the Arduino
2. Upload Arduino firmware (moveo_moveit/moveo_moveit_arduino/moveo_moveit_arduino.ino)
3. Create a virtualenv that has Python 3, OpenCV 3, and Tensorflow 1.2+
4. Within that virtualenv (in moveo_ros/object_detector_app), in terminal run: ```python object_detection_multithreading.py```
5. In another terminal (no virtualenv), run: ``` roscore ```
5. In another terminal (no virtualenv), run: ``` rosrun rosserial_python serial_node.py /dev/ttyUSB0 ``` (establishes rosserial node that communicates with Arduino)
6. Tweak your predefined object-specific trajectories (for now, this is not so robust, but I'm working on it!)
7. In another terminal (no virtualenv), run: ``` rosrun moveo_moveit moveo_objrec_publisher.py ```
8. Now, when object is placed in the FOV of the camera, a trajectory will be performed based on what object is detected! In this example, I've set up the framework to perform a sequence of trajectories that together form different 'pick and place' trajectories for each object.

## How it Works
### Real-Time Object Recognition
Here, we use Python 3, Tensorflow's Object Detection API, OpenCV, and an ordinary webcam to create this application.  For more information about how this works, see [here](https://github.com/jesseweisberg/moveo_ros/tree/master/object_detector_app).  While real-time object recognition is going on, the label of the recognized object is sent via ZMQ to a node in ROS.  Then, the node in ROS publishes to a rostopic.  The intermedicary step of sending via ZMQ is necessary because ROS only supports Python 2, whereas the object recognition app uses Python 3.  Thus, ZMQ really is just used to send the detected object info from a Python 3 (non-ROS friendly) environment to a Python 2 (ROS friendly) environment.

### Using Real-Time Object Recognition to Perform Object-Specific Pick and Place
The node, **moveo_objrec_publisher.py** (in moveo_ros/moveo_moveit/scripts), receives the label of the recognized object from the object_detection_multithreading.py script, and publishes a sequence of trajectories (on the /joint_steps topic) to perform a 'pick and place' motion for that specific object.  The Arduino subscribes to the /joint_steps topic and performs the trajectories.
