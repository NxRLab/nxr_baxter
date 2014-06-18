#!/usr/bin/env python

# Adam Barber
# June 2014

# Node for testing moveit stuff

import rospy
import moveit_commander
import moveit_msgs.msg

from baxter_core_msgs.srv import (
    SolvePositionIK
    SolvePositionIKRequest)

from geometry_msgs.msg import (
    PoseStamped
    Pose)

from tf.transformations import createQuaternionFromRPY

if __name__=='__main__':
    rospy.loginfo("Starting test_moveit node")
    rospy.init_node('test_moveit', log_level=rospy.INFO)

    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    pose = [0.28, -0.62, -0.32, 3.14/4, -3.14/2, 0]
    # pose = [0.815, -1.01, 0.321, 0.271, 0.653, -0.271, 0.653]

    limb = "left_right"
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    posestamp = Pose(
        position=Point(pose[0:3]),
        orienation=createQuaternionFromRPY(pose[3:])
    )
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.loginfo("Service exception")

    if resp.result_type[0] != resp.RESULT_INVALID:
        des_joints = resp.joints[0];

        print right_arm_group.get_current_pose()

        right_arm_group.set_joint_value_target(des_joints)

        right_arm_group.plan()
        right_arm_group.move()
    
        print right_arm_group.get_current_pose()

    else:
        print "IK results were not valid."
