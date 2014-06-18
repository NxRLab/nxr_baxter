#!/usr/bin/env python

# Adam Barber
# June 2014

# Node for testing moveit stuff

import rospy
import moveit_commander
import moveit_msgs.msg

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion)

from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    rospy.loginfo("Starting test_moveit node")
    rospy.init_node('test_moveit', log_level=rospy.INFO)

    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    # des_pose = [0.28, -0.62, -0.32, 0, -3.14/2, 0]
    des_pose = [0.815, -1.01, 0.321, 0.271, 0.653, -0.271, 0.653]

    limb = "right"
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # quat = quaternion_from_euler(des_pose[3],des_pose[4],des_pose[5])
    pose = Pose()
    quat = Quaternion()
    quat.x = des_pose[3]
    quat.y = des_pose[4]
    quat.z = des_pose[5]
    quat.w = des_pose[6]
    pose.orientation = quat
    pose.position.x = des_pose[0]
    pose.position.y = des_pose[1]
    pose.position.z = des_pose[2]
    ikreq.pose_stamp = [PoseStamped(header=hdr, pose=pose)]
    # print ikreq
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.loginfo("Service exception")

    if resp.isValid[0]:
        des_joints = [0]*7
        for i in range(7):
            des_joints[i] = resp.joints[0].position[i]
        # print right_arm_group.get_current_pose()
        # print des_joints
        right_arm_group.set_joint_value_target(des_joints)

        right_arm_group.plan()
        right_arm_group.go()
    
        print right_arm_group.get_current_pose()

    else:
        print right_arm_group.get_current_pose()
        print "IK results were not valid."
