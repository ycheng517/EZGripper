#! /usr/bin/env python

import sys, thread, time, sched
import rospy
from control_msgs.msg import GripperCommandActionGoal
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math

def callback(data):
    print("Got a data: %f" % data)

if __name__ == "__main__":
    # main()
    seq_id = 0
    rospy.init_node("EZG_Interceptor")
    sub = rospy.Subscriber('/joint_states', JointState, callback)
    
    # hand_1 = rospy.Publisher('/ur3x/knuckle_1_position_controller/command', Float64, queue_size=10)
    # hand_2 = rospy.Publisher('/ur3x/knuckle_2_position_controller/command', Float64, queue_size=10)
    
    
    while not rospy.is_shutdown():
        rospy.sleep(0.02)
