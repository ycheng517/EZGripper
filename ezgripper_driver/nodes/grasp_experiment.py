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
parser.add_argument('--sdf', dest='sdf', action='store_true')
parser.add_argument('--urdf', dest='urdf', action='store_true')
parser.set_defaults(sdf=False)
parser.set_defaults(urdf=False)

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
    rospy.sleep(2)

    # spawn object
    if args.urdf: 
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        s = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    if args.sdf: 
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        s = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    orient = Quaternion(*tf.transformations.quaternion_from_euler(obj_pose[3], obj_pose[4], obj_pose[5]))
    obj_pose_q = Pose(Point(obj_pose[0], obj_pose[1], obj_pose[2]), orient)
    obj_file = dirname(dirname(abspath(__file__))) + "/urdf/" + args.object_file_name
    with open(obj_file, "r") as f:
        obj_xml = f.read()

    print "model spawn " + str(s("grasp_object", obj_xml, "", obj_pose_q, "world"))
    rospy.sleep(0.02)

    # get object height (for grasp verification later)
    res = gms_client("grasp_object","world")
    prev_obj_height = res.pose.position.z

    # close gripper
    msg = Float64()
    msg.data = -1.5
    gripper_l.publish(msg)
    rospy.sleep(0.02)
    gripper_r.publish(msg)
    rospy.sleep(0.02)
    gripper_ref.publish(msg)
    rospy.sleep(2)

    # lift gripper in z direction
    msg.data = gripper_pose_args[2] + 0.2
    base_z.publish(msg)
    rospy.sleep(1)
    # check if grasp is successful
    res = gms_client("grasp_object","world")
    if res.pose.position.z - prev_obj_height > 0.15:
        print("grasp success, z clearance is: %f" % (res.pose.position.z - prev_obj_height))
    else: 
        print("grasp failed, z clearance is: %f" % (res.pose.position.z - prev_obj_height))
    
