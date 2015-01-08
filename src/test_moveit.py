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

def move_pos(des_pose):
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

    return 

    
    
if __name__=='__main__':
    rospy.loginfo("Starting test_moveit node")
    rospy.init_node('test_moveit', log_level=rospy.INFO)

    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    # des_pose = [0.28, -0.62, -0.32, 0, 3.14/2, 0]
    des_pose = [0.815, -1.01, 0.321, 0.271, 0.653, -0.271, 0.653]
    # des_pose2 = [0.724, 0.125, -0.055, 0.183, 0.983, -0.013, -0.015]
    des_pose2 = [1.037, 0.140, 0.212, -0.072, 0.744, 0.041, 0.663]

    # add scene:
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    rospy.sleep(3.0)
    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.75
    p.pose.position.y = -0.25
    p.pose.position.z = 0.1

    scene.remove_world_object("table")
    scene.add_box("collision_box", p, (0.25, 0.1, 1.0))

    # p = PoseStamped()
    # p.header.frame_id = robot.get_planning_frame()
    # p.pose.position.x = 0.8
    # p.pose.position.y = 0.025
    # p.pose.position.z = -0.6
    # scene.add_box("table", p, (0.75, 1.25, 0.68))


    move_pos(des_pose)

    rospy.sleep(2)

    move_pos(des_pose2)

    
