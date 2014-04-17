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
from trajectory_msgs.msg import JointTrajectoryPoint

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

import baxter_interface
import baxter_dataflow
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
# from std_msgs.msg import Header
from nxr_baxter_msgs.srv import *

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

        self.left_joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                                 'left_w0', 'left_w1', 'left_w2']
        self.right_joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                                 'right_w0', 'right_w1', 'right_w2']

        ns_left = 'robot/limb/left/'
        ns_right = 'robot/limb/right/'

        self.left_client = actionlib.SimpleActionClient(
            ns_left + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self.right_client = actionlib.SimpleActionClient(
            ns_right + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        self.left_goal = FollowJointTrajectoryGoal()
        self.right_goal = FollowJointTrajectoryGoal()
        server_up_left = self.left_client.wait_for_server(timeout=rospy.Duration(10.0))
        server_up_right = self.right_client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up_left or not server_up_right:
            rospy.logerr("Timeout waiting for JTAS")
            rospy.signal_shutdown("Timed out waiting...")
            sys.exit(1)

        # self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        # self.pub_rate.publish(500)
        self.new_traj = False
        self.traj = None

        # self.left_goal.trajectory = self.split_traj(traj, 0, 7)
        # self.right_goal.trajectory = self.split_traj(traj, 7, 0)
        # self.left_goal.trajectory.header.stamp = rospy.Time.now()
        # self.right_goal.trajectory.header.stamp = rospy.Time.now()
        # for i in range(len(self.left_goal.trajectory.points)):
        #     self.left_goal.trajectory.points[i].time_from_start = rospy.Duration(self.left_goal.trajectory.points[i].time_from_start)
        #     self.right_goal.trajectory.points[i].time_from_start = rospy.Duration(self.right_goal.trajectory.points[i].time_from_start)
        # self.left_goal.trajectory.joint_names = self.left_joint_names
        # self.right_goal.trajectory.joint_names = self.right_joint_names
        # self.left_client.send_goal(self.left_goal)
        # self.right_client.send_goal(self.right_goal)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback_start, oneshot=True)
        # while not rospy.is_shutdown():
        #     self.timer_callback()
        #     rospy.sleep(0.001)

    def timer_callback_start(self, event):
        while not rospy.is_shutdown():
            self.timer_callback()
            rospy.sleep(0.001)

    def timer_callback(self):
        if self.traj != None and self.new_traj:
            # Find the time in the trajectory
            time_from_start = rospy.get_time() - self.traj.header.stamp.to_sec()
            # Send the points at that time to move
            traj_to_send = self.interpolate_trajectory(time_from_start)
            if traj_to_send == None:
                rospy.logwarn("timer_callback")
                self.new_traj = False
            else:
                # joints = {"left": [], "right": []}
                self.left_goal = FollowJointTrajectoryGoal()
                self.right_goal = FollowJointTrajectoryGoal()
                self.left_goal.trajectory = self.split_traj(traj_to_send,0,7)
                self.right_goal.trajectory = self.split_traj(traj_to_send,7,0)
                # joints["left"] = traj_point.positions[0:7]
                # joints["right"] = traj_point.positions[7:]
                # self._goal.trajectory.header.stamp = rospy.Time.now()
                self.left_goal.trajectory.header.stamp = rospy.Time.now()
                self.right_goal.trajectory.header.stamp = rospy.Time.now()
                self.left_goal.trajectory.joint_names = self.left_joint_names
                self.right_goal.trajectory.joint_names = self.right_joint_names
                # rospy.logerr("%f", self.left_goal.trajectory.header.stamp.secs)
                # rospy.logerr("%f", self.right_goal.trajectory.header.stamp.secs)
                self.left_client.send_goal(self.left_goal)
                self.right_client.send_goal(self.right_goal)
                self.left_client.wait_for_result(timeout=rospy.Duration(5.0))
                self.right_client.wait_for_result(timeout=rospy.Duration(5.0))
                # self.move(joints)
        self.new_traj = False


    def split_traj(self, traj, start, end):
        new_traj = deepcopy(traj)
        if end == 0:
            for i in range(len(new_traj.points)):
                new_traj.points[i].positions = new_traj.points[i].positions[start:]
                new_traj.points[i].velocities = new_traj.points[i].velocities[start:]
                new_traj.points[i].accelerations = new_traj.points[i].accelerations[start:]
        else:
            for i in range(len(new_traj.points)):
                new_traj.points[i].positions = new_traj.points[i].positions[start:end]
                new_traj.points[i].velocities = new_traj.points[i].velocities[start:end]
                new_traj.points[i].accelerations = new_traj.points[i].accelerations[start:end]
        return new_traj

    def push(self, new_plan):
        self.new_traj = True
        self.traj = deepcopy(new_plan.joint_trajectory)
    
    def interpolate_trajectory(self, time_from_start):
        if len(self.traj.points) == 0:
            rospy.loginfo("Traj length is 0")
            return None
        for j in range(len(self.traj.points)):
            point = self.traj.points[j]
            # print "time_from_start: ", type(time_from_start)
            # print "point.time_from_start: ", type(point.time_from_start)
            if type(time_from_start) != type(point.time_from_start):
                print "Time from start: ", type(time_from_start)
                print "Point Time from Start: ", type(point.time_from_start)
            if time_from_start <= point.time_from_start:
                # We are passed our time, good sign, now we interpolate
                if j == 0:
                    # First point is after us, lets use that one
                    rospy.logwarn("Somehow the first point is in the future")
                    return self.traj
                else:
                    self.traj.points = self.traj.points[j:]
                    temp_time = self.traj.points[0].time_from_start.to_sec()
                    for i in range(len(self.traj.points)):
                        self.traj.points[i].time_from_start -= temp_time
                        self.traj.points[i].time_from_start = rospy.Duration(self.traj.points[i].time_from_start)
                    return self.traj
        self.traj.points = [self.traj.points[-1]]
        self.traj.points[0].time_from_start = rospy.Duration(0.0)
        return self.traj

if __name__=='__main__':
    rospy.loginfo("Starting moveit_interface node")
    rospy.init_node('moveit_interface', log_level=rospy.INFO)

    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    scene = moveit_commander.PlanningSceneInterface()

    robot = moveit_commander.RobotCommander()
    rospy.loginfo("I'm sitting here.")
    rospy.sleep(3.0)
    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    # Table
    # p.pose.position.x = 1.0
    # p.pose.position.y = 0.0
    # p.pose.position.z = 0.0
    # scene.add_box('table', p, (1, 0.1, 2))
    p.pose.position.x = 1.0
    p.pose.position.y = 0.025
    p.pose.position.z = -0.6
    scene.add_box("table", p, (0.8, 1.25, 0.8))
    # scene.add_box("table", p, (0.7, 1.25, 0.71))
    # p.pose.position.x = 0.0
    # p.pose.position.y = -52.5*2.54/100.0
    # p.pose.position.z = 0.0
    # scene.add_plane("right_wall", p, normal=(0, 1, 0))
    # p.pose.position.y = 54*2.54/100.0
    # scene.add_plane("left_wall", p, normal=(0, -1, 0))

    # both_arms_group.set_goal_joint_tolerance(0.05)

    traj_controller = Trajectory_Controller()

    while not rospy.is_shutdown():
        rospy.wait_for_service('joint_values')
        try:
            joint_service_proxy = rospy.ServiceProxy('joint_values', JointValues)
            joint_resp = joint_service_proxy()
            if joint_resp.new_values:
                joints = dict(zip(joint_resp.joint_names,
                                  joint_resp.joint_values))
                try:
                    traj = both_arms_group.plan(joints)
                    traj_controller.push(traj)
                except moveit_commander.MoveItCommanderException, e:
	                rospy.logwarn("Error setting joint target, target not within bounds.")
            else:
                rospy.logdebug('Got old values')
                rospy.sleep(0.001)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s', e)

    rospy.loginfo("moveit_interface shutting down")
