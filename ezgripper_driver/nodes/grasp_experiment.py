#! /usr/bin/env python

import sys, rospy, tf, argparse
import thread, time, sched
from os.path import dirname, abspath
from std_msgs.msg import Float64
from gazebo_msgs.srv import *
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import *
from decimal import Decimal

parser = argparse.ArgumentParser(description='specify arguments for grasp experiment')
parser.add_argument('--gripper_pose', dest='gripper_pose', nargs=6, help='<required> x y z angle_x angle_y angle_z', required=True)
parser.add_argument('--object_pose', dest='object_pose', nargs=6, help='<required> x y z angle_x angle_y angle_z', required=True)
parser.add_argument('--object_file_name', dest='object_file_name', help='<required> file name of the object, not including path', required=True)

def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    seq_id = 1
    rospy.init_node("grasp_experiment")
    args = parser.parse_args()
    obj_pose = [float(i) for i in args.object_pose]
    gripper_pose_args = [float(i) for i in args.gripper_pose]

    base_x = rospy.Publisher('/base_x_controller/command', Float64, queue_size=5)
    base_y = rospy.Publisher('/base_y_controller/command', Float64, queue_size=5)
    base_z = rospy.Publisher('/base_z_controller/command', Float64, queue_size=5)
    base_rx = rospy.Publisher('/base_rx_controller/command', Float64, queue_size=5)
    base_ry = rospy.Publisher('/base_ry_controller/command', Float64, queue_size=5)
    base_rz = rospy.Publisher('/base_rz_controller/command', Float64, queue_size=5)
    gripper_l = rospy.Publisher('/left_controller/command', Float64, queue_size=5)
    gripper_r = rospy.Publisher('/right_controller/command', Float64, queue_size=5)
    gripper_ref = rospy.Publisher('/spring_calculation_controller/command', Float64, queue_size=5)

    rospy.sleep(1)
    # open gripper
    msg = Float64()
    msg.data = 0.0
    gripper_l.publish(msg)
    rospy.sleep(0.05)
    gripper_r.publish(msg)
    rospy.sleep(0.05)
    gripper_ref.publish(msg)
    rospy.sleep(0.05)
    rospy.sleep(3)

    #move gripper in position
    msg = Float64()
    msg.data = gripper_pose_args[0]
    base_x.publish(msg)
    rospy.sleep(0.02)
    msg.data = gripper_pose_args[1]
    base_y.publish(msg)
    rospy.sleep(0.02)
    msg.data = gripper_pose_args[2]
    base_z.publish(msg)
    rospy.sleep(0.02)
    msg.data = gripper_pose_args[3]
    base_rx.publish(msg)
    rospy.sleep(0.02)
    msg.data = gripper_pose_args[4]
    base_ry.publish(msg)
    rospy.sleep(0.02)
    msg.data = gripper_pose_args[5]
    base_rz.publish(msg)
    rospy.sleep(3)

    # spawn object
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    s = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    orient = Quaternion(*tf.transformations.quaternion_from_euler(obj_pose[3], obj_pose[4], obj_pose[5]))
    obj_pose_q = Pose(Point(obj_pose[0], obj_pose[1], obj_pose[2]), orient)
    obj_file = dirname(dirname(abspath(__file__))) + "/urdf/" + args.object_file_name
    with open(obj_file, "r") as f:
        obj_xml = f.read()

    print s("grasp_object", obj_xml, "", obj_pose_q, "world")
    rospy.sleep(1)

    # get object height (for grasp verification later)
    res = gms_client("grasp_object","world")
    prev_obj_height = res.pose.position.z

    # close gripper
    msg = Float64()
    msg.data = -0.2
    gripper_l.publish(msg)
    rospy.sleep(0.02)
    gripper_r.publish(msg)
    rospy.sleep(0.02)
    gripper_ref.publish(msg)
    rospy.sleep(3)

    # lift gripper in z direction
    msg.data = gripper_pose_args[2] + 0.2
    base_z.publish(msg)

    # check if grasp is successful
    res = gms_client("grasp_object","world")
    if res.pose.position.z - prev_obj_height > 0.2:
        print("grasp success")
    else: 
        print("grasp failed")
    
