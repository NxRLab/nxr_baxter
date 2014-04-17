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

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import baxter_interface
import baxter_dataflow
from geometry_msg.msg import PoseStamped
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

        # self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        # self.pub_rate.publish(500)
        self.traj = None
        rospy.Timer(rospy.Duration(0.25), self.timer_callback, oneshot=False)

    def timer_callback(self, event):
        if self.traj != None:
            # Find the time in the trajectory
            time_from_start = rospy.get_time() - self.traj.header.stamp.to_sec()
            # Send the points at that time to move
            traj_point = self.interpolate_trajectory(time_from_start)
            if traj_point == None:
                rospy.logwarn("timer_callback")
                return
            joints = {"left": [], "right": []}
            joints["left"] = traj_point.positions[0:7]
            joints["right"] = traj_point.positions[7:]
            self.move(joints)
        else:
            return

    def move_thread(self, limb, position, queue, timeout=15.0):
        if limb == 'left':
            l_positions = dict(zip(self.left_arm.joint_names(), position[0:7]))
            self.left_arm.set_joint_positions(l_positions)
        elif limb == 'right':
            r_positions = dict(zip(self.right_arm.joint_names(), position[0:7]))
            self.right_arm.set_joint_positions(r_positions)

    def move(self, joints):
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

    def push(self, new_traj):
        self.traj = new_traj.joint_trajectory
    
    def interpolate_trajectory(self, time_from_start):
        if len(self.traj.points) == 0:
            return None
        for j in range(len(self.traj.points)):
            point = self.traj.points[j]
            if time_from_start <= point.time_from_start:
                # We are passed our time, good sign, now we interpolate
                if j == 0:
                    # First point is after us, lets use that one
                    rospy.logwarn("Somehow the first point is in the future")
                    return point
                else:
                    prev_point = self.traj.points[j-1]
                    for i in range(len(point.positions)):
                        point.positions[i] = (point.position[i] - prev_point.position[i])*(time_from_start - prev_point.time_from_start.to_sec())/(point.time_from_start.to_sec() - prev_point.time_from_start.to_sec())
                    self.traj.points = self.traj.points[j:]
                    return point
        self.traj.points = [self.traj.points[-1]]
        return point

if __name__=='__main__':
    rospy.loginfo("Starting moveit_interface node")
    rospy.init_node('moveit_interface', log_level=rospy.INFO)

    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    planning_scene = moveit_commander.PlanningSceneInterface()

    rospy.loginfo('Sitting here.')
    rospy.sleep(3.0)

    robot = moveit_commander.RobotCommander()

    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = 1.0
    p.pose.position.y = 0.025
    p.pose.position.z = -0.6
    scene.add_box("table", p, (0.8, 1.25, 0.8))
    # # Table
    # p.pose.position.x = (23.5 + 4.25 + 12)*2.54/100.0
    # p.pose.position.y = 0.0
    # p.pose.position.z = (-13.0625)*2.54/100.0
    # scene.add_box("table", p, (24*2.54/100.0, 48*2.54/100.0, 1.125*2.54*100))
    # p.pose.position.x = 0.0
    # p.pose.position.y = -52.5*2.54/100.0
    # p.pose.position.z = 0.0
    # scene.add_plane("right_wall", p, normal=(0, 1, 0))
    # p.pose.position.y = 54*2.54/100.0
    # scene.add_plane("left_wall", p, normal=(0, -1, 0))

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
                    both_arms_group.set_joint_value_target(joints)
                    traj_controller.push(both_arms_group.plan())
                except MoveitCommanderException, e:
	                rospy.logwarn("Error setting joint target, target not within bounds.")
                # both_arms_group.go()
            else:
                rospy.logdebug('Got old values')
                rospy.sleep(0.001)
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s', e)

    rospy.loginfo("moveit_interface shutting down")
