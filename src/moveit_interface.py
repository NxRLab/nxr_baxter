#!/usr/bin/env python

# Adam Barber
# April 2014

# Node for interfacing with moveit

import rospy
import sys
import Queue
import threading
import moveit_commander
import moveit_msgs.msg

import actionlib

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import (JointTrajectoryPoint, JointTrajectory)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest)

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

import baxter_interface
import baxter_dataflow

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Quaternion)
# from std_msgs.msg import Header
from nxr_baxter_msgs.srv import *

from tf.transformations import quaternion_from_euler

class Trajectory_Controller:
    """
    Trajectory Controller

    This class takes in new trajectories whenever they are available, and
    continues to use the old-style controller of calling each arm by itself
    with threading.
    """

    def __init__(self):
        rospy.logdebug("Calling Trajectory_Controller.__init__()")
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        self.left_arm.set_joint_position_speed(0.4)
        self.right_arm.set_joint_position_speed(0.4)

        self.traj = None

        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback, oneshot=False)


    def timer_callback(self,event):
        if self.traj != None:
            # Find the time in the trajectory
            time_from_start = rospy.get_time() - self.traj.header.stamp.to_sec()
            # Send the points at that time to move
            traj_point = self.interpolate_trajectory(time_from_start)
            if traj_point == None:
                # rospy.logwarn("timer_callback")
                return
            joints = {"left": [], "right": []}
            joints["left"] = traj_point.positions[0:7]
            joints["right"] = traj_point.positions[7:]
            self.move(joints)

    def push(self, new_plan):
        # self.new_traj = True
        self.traj = new_plan.joint_trajectory
        

    def push_one_arm(self, new_plan, left_arm_joints):
        # Build a new joint trajectory with left_arm_joint values at those times
        new_traj = new_plan.joint_trajectory
        temp_traj = JointTrajectory()
        temp_traj.points = [JointTrajectoryPoint()]*len(new_traj.points)
        temp_traj.header = new_traj.header
        temp_traj.joint_names = self.traj.joint_names
        zero_vec = [0.0]*7
        left_arm_vals = [0.35, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0]
        for i in range(len(new_traj.points)):
            temp_traj.points[i].positions = left_arm_vals + list(new_traj.points[i].positions)
            temp_traj.points[i].velocities = zero_vec + list(new_traj.points[i].velocities)
            temp_traj.points[i].accelerations = zero_vec + list(new_traj.points[i].accelerations)
            temp_traj.points[i].time_from_start = new_traj.points[i].time_from_start
        self.traj = temp_traj
    
    def interpolate_trajectory(self, time_from_start):
        if len(self.traj.points) == 0:
            rospy.logdebug("Traj length is 0")
            return None
        for j in range(len(self.traj.points)):
            point = self.traj.points[j]
            # print "time_from_start: ", type(time_from_start)
            # print "point.time_from_start: ", type(point.time_from_start)
            time_to_use = point.time_from_start
            if type(time_from_start) != type(time_to_use):
                time_to_use = time_to_use.to_sec()
            if time_from_start <= time_to_use:
                # We are passed our time, good sign, now we interpolate
                if j == 0:
                    # First point is after us, lets use that one
                    rospy.logwarn("Somehow the first point is in the future")
                    return traj.points[i]
                else:
                    self.traj.points = self.traj.points[j:]
                    time_offset = self.traj.points[i].time_from_start
                    for i in range(len(self.traj.points)):
                        self.traj.points[i].time_from_start -= time_offset
                    return self.traj.points[j]
        self.traj.points = [self.traj.points[-1]]
        self.traj.points[0].time_from_start = rospy.Duration(0.0)
        return self.traj.points[0]

    def move_thread(self, limb, position, queue, timeout=15.0):
        if limb == 'left':
            l_positions = dict(zip(self.left_arm.joint_names(), position[0:7]))
            self.left_arm.set_joint_positions(l_positions)
        elif limb == 'right':
            r_positions = dict(zip(self.right_arm.joint_names(), position[0:7]))
            self.right_arm.set_joint_positions(r_positions)

    def move(self, joints):
        # rospy.loginfo('move')
        left_queue = Queue.Queue()
        right_queue = Queue.Queue()
        left_thread = threading.Thread(target=self.move_thread,
                                       args=('left', joints['left'], left_queue))
        right_thread = threading.Thread(target=self.move_thread,
                                       args=('right', joints['right'], right_queue))
        left_thread.daemon = True
        right_thread.daemon = True
        left_thread.start()
        right_thread.start()
        baxter_dataflow.wait_for(
            lambda: not (left_thread.is_alive() or
                         right_thread.is_alive()),
            timeout=20.0,
            timeout_msg=("Timeout while waiting for arm move threads"
                         " to finish"),
            rate=10,
        )
        left_thread.join()
        right_thread.join()

class Inverse_Kinematics:
    """
    This class handles inverse kinematics for this script. It allows easy
    accessability to the IKservice that Rethink provides.
    """
    def __init__(self,limb="right"):
        rospy.logdebug("Calling Inverse_Kinematics.__init__()")
        self.ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        
    def solveIK(self,des_pose):
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = Pose()
        quat = Quaternion()
        if len(des_pose) == 6:
            #Note that this may be wrong!
            quatArray = quaternion_from_euler(des_pose[3], des_pose[4], des_pose[5], axes='sxyz')
            quat.x = quatArray[0]
            quat.y = quatArray[1]
            quat.z = quatArray[2]
            quat.w = quatArray[3]
        else:
            quat.x = des_pose[3]
            quat.y = des_pose[4]
            quat.z = des_pose[5]
            quat.w = des_pose[6]
        pose.orientation = quat
        pose.position.x = des_pose[0]
        pose.position.y = des_pose[1]
        pose.position.z = des_pose[2]
        # print pose.position

        ikreq.pose_stamp = [PoseStamped(header=hdr, pose=pose)]
        try:
            rospy.wait_for_service(self.ns,5.0)
            resp = self.iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("IK Service Exception thrown")
            return None

        if resp.isValid[0]:
            print resp.isValid
            des_joints = [0]*7
            for i in range(7):
                des_joints[i] = resp.joints[0].position[i]
            return des_joints
        else:
            # print resp.isValid
            return None

if __name__=='__main__':
    rospy.init_node('moveit_interface', log_level=rospy.INFO)
    rospy.loginfo("Starting moveit_interface node")

    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    # both_arms_group.allow_replanning(True)
    # right_arm_group.allow_replanning(True)
    # left_arm_group.allow_replanning(True)
    both_arms_group.set_goal_position_tolerance(0.01)
    both_arms_group.set_goal_orientation_tolerance(0.01)
    left_arm_group.set_goal_position_tolerance(0.01)
    left_arm_group.set_goal_orientation_tolerance(0.01)
    right_arm_group.set_goal_position_tolerance(0.01)
    right_arm_group.set_goal_orientation_tolerance(0.01)

    #Try setting workspace bounds, instead of maybe checking joint limits.
    both_arms_group.set_workspace([-10, -51*2.54/100.0, -10, 10, 53*2.54/100.0, 10])
    right_arm_group.set_workspace([-10, -51*2.54/100.0, -10, 10, 53*2.54/100.0, 10])
    left_arm_group.set_workspace([-10, -51*2.54/100.0, -10, 10, 53*2.54/100.0, 10])

    scene = moveit_commander.PlanningSceneInterface()

    rospy.loginfo('Sitting here.')
    rospy.sleep(3.0)

    robot = moveit_commander.RobotCommander()

    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.8
    p.pose.position.y = 0.0
    p.pose.position.z = -0.70
    scene.add_box("table", p, (0.762, 1.239, 1.0))

    # add right splash guard
    p.pose.position.x = 0.8
    p.pose.position.y = -1.239/2.0
    p.pose.position.z = -.135
    scene.add_box("right_guard", p, (0.762, 0.025, 0.127))


    # add left splash guard
    p.pose.position.x = 0.8
    p.pose.position.y = 1.239/2.0
    p.pose.position.z = -.135
    scene.add_box("left_guard", p, (0.762, 0.025, 0.127))


    # add back splash guard
    p.pose.position.x = 0.8 - .762/2
    p.pose.position.y = 0.0
    p.pose.position.z = -.135
    scene.add_box("back_guard", p, (.025, 1.239, 0.127))


    traj_controller = Trajectory_Controller()

    ikright = Inverse_Kinematics(limb="right")

    right_arm = baxter_interface.Limb('right')
    left_arm = baxter_interface.Limb('left')

    while not rospy.is_shutdown():
        rospy.wait_for_service('joint_values')
        try:
            joint_service_proxy = rospy.ServiceProxy('joint_values', JointValues)
            joint_resp = joint_service_proxy()
            if joint_resp.new_values:
                if "joints" in joint_resp.val_type:
                    joints = dict(zip(joint_resp.joint_names,
                                      joint_resp.joint_values))
                    try:
			print "Setting Joint Values"
                        both_arms_group.set_joint_value_target(joints)
                        traj_controller.push(both_arms_group.plan())
                    except moveit_commander.MoveItCommanderException, e:
                        rospy.logwarn("Error setting joint target, target not within bounds.")
                elif "pose" in joint_resp.val_type:
                    try:
                        joints = dict(zip(joint_resp.joint_names,
                                          joint_resp.joint_values))
                        pose = [joints['x'], joints['y'], joints['z'],
                                joints['yaw'], joints['pitch'], joints['roll']]
                        desired_joints = ikright.solveIK(pose)
                        #print "Desired Joints: ", desired_joints
                        if desired_joints != None:
                            joints_one_arm = dict(zip(right_arm.joint_names(),
                                                      desired_joints))
                            right_arm_group.set_joint_value_target(joints_one_arm)
                            traj_controller.push_one_arm(right_arm_group.plan(), left_arm.joint_angles())
                    except moveit_commander.MoveItCommanderException, e:
                        rospy.logwarn("Error setting cartesian pose target.")
                    
            else:
                rospy.logdebug('Got old values')
                rospy.sleep(0.001)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s', e)
 
    rospy.loginfo("moveit_interface shutting down")
