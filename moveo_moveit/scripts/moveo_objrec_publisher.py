#!/usr/bin/env python
#!/usr/bin/env python3

'''
Subscribes to a zmq socket and publishes that information to a ros topic.  This is one workaround for using
Python 2 and Python 3 in the same ROS application.

In my case, this receives real-time object detection info from a script in Python 3 and publishes to a rostopic.

Author: Jesse Weisberg
'''
import rospy
from std_msgs.msg import String
import sys
import zmq
from msgpack import loads
import time
import pyttsx
from datetime import datetime 
from espeak import espeak
from moveo_moveit.msg import ArmJointState

fixated_object_label = None
gripper = {'open': 0, 'banana': 70, 'apple': 50}
upright = [0, 0, 0, 0, 0, 0]

#predefined movements for pick and place of an apple and banana
apple_pick = [0, -2243, -23410, 14, -800, gripper['apple']]
apple_move = [0, -1113, -17410, 14, -3300, gripper['apple']]
apple_place = [-4600, -2400, -18410, 91, -800, gripper['open']]

banana_pick = [0, -2243, -24410, 14, -400, gripper['banana']]
banana_move = [0, -1043, -17410, 14, -3300, gripper['banana']]
banana_place = [4600, -2400, -20410, -91, -400, gripper['open']]


object_trajectories = {"apple": [upright, apple_pick, apple_move, apple_place, upright],
                       "banana": [upright, banana_pick, banana_move, banana_place, upright]}


#subscribe to detected object from object_detection_pupil.py (Pupil object detection plugin) via zmq
def subscribe_detected_object():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    addr = '127.0.0.1'  # remote ip or localhost
    port = "5556"  # same as in the pupil remote gui
    print('retrieving objects...')
    socket.connect("tcp://{}:{}".format(addr, port))

    #subscribe to detected_objects topic
    while True:
        try:
            socket.setsockopt_string(zmq.SUBSCRIBE, 'detected_object')
        except TypeError:
            socket.setsockopt(zmq.SUBSCRIBE, 'detected_object')
        #process object
        detected_object = socket.recv_string() 
        if len(detected_object.split())==3:
            fixated_object_label = detected_object.split()[1]
            confidence = detected_object.split()[2]
        if len(detected_object.split())==4:
            fixated_object_label = detected_object.split()[1] + ' ' + detected_object.split()[2]
            confidence = detected_object.split()[3]

        # Potential improvement idea with emg sensory feedback
        # activate grasp for robotic manipulator: turn on "ready to execute switch"
        # time.sleep(3), during this time wait for emg sensory input
        # set up another rostopic that with emg sensory input, 
        # arduino reads that if higher than thresh, execute predetermined motion planning/grasp  
        return fixated_object_label


# publish detected object to a ros topic
def publish_detected_object():
    pub = rospy.Publisher('joint_steps', ArmJointState, queue_size=4)
    rospy.init_node('pick_and_place_object_detection', anonymous=True)
    rate = rospy.Rate(.1) # 20hz

    while not rospy.is_shutdown():
        fixated_object_label = subscribe_detected_object()
        rospy.loginfo(fixated_object_label)
        
        # check if fixated object label is a key in object_trajectories
        # if so, publish each trajectory in object_trajectories[key] to ArmJointState
        if fixated_object_label in object_trajectories:
            for i in object_trajectories[fixated_object_label]:
                goal = ArmJointState()
                goal.position1 = i[0]
                goal.position2 = i[1]
                goal.position3 = i[2]
                goal.position4 = i[3]
                goal.position5 = i[4]
                goal.position6 = i[5]
                pub.publish(goal)
                rospy.sleep(10)
                
        espeak.synth(fixated_object_label)
        while espeak.is_playing():
             pass

        #rate.sleep()
    

if __name__ == '__main__':
    try:
        publish_detected_object()
    except rospy.ROSInterruptException:
        pass
