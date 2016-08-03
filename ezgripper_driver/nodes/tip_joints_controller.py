#! /usr/bin/env python

import sys, thread, time, sched
import rospy
from control_msgs.msg import GripperCommandActionGoal
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math

knuckle_1 = 0
knuckle_2 = 1
tip_joint_1 = 2
tip_joint_2 = 3
ref_joint = 4

def callback(data):
    if (abs(data.position[knuckle_1] - data.position[ref_joint]) > 0.05):
        msg = Float64()
        msg.data = -(data.position[knuckle_1] - data.position[ref_joint])
        if msg.data < -0.5:
            msg.data = -0.5
        finger_1.publish(msg)
        print("tip joint 1 effort: %f" % msg.data)
    else: 
        msg = Float64()
        msg.data = 0
        finger_1.publish(msg)

    if (abs(data.position[knuckle_2] - data.position[ref_joint]) > 0.05):
        msg = Float64()
        
        msg.data = -(data.position[knuckle_2] - data.position[ref_joint]) 
        if msg.data < -0.5:
            msg.data = -0.5
        finger_2.publish(msg)
        print("tip joint 2 effort: %f" % msg.data)
    else: 
        msg = Float64()
        msg.data = 0
        finger_2.publish(msg)

if __name__ == "__main__":
    # main()
    seq_id = 0
    rospy.init_node("EZG_Interceptor")
    sub = rospy.Subscriber('/joint_states', JointState, callback)
    
    finger_1 = rospy.Publisher('/left_tip_controller/command', Float64, queue_size=5)
    finger_2 = rospy.Publisher('/right_tip_controller/command', Float64, queue_size=5)
    finger_ref = rospy.Publisher('/spring_calculation_controller/command', Float64, queue_size=5)
    
    
    while not rospy.is_shutdown():
        rospy.sleep(1)
