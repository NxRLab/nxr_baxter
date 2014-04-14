#!/usr/bin/env python

# Adam Barber April 2014

# Contains functions to handle speeding up (slowing down) of trajectories for
# use later in moveit.

# Taken from pirobot's post here: https://github.com/ros-planning/moveit_ros/issues/368


################
#  ROS IMPORTS #
################
import rospy

####################
# OTHER IMPORTS: #
####################
import numpy as np

##################
# MOVEIT Imports #
##################
from moveit_msgs.msg import RobotTrajectory
# from moveit_commander import RobotTrajectory, JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectoryPoint


def traj_speed_up(traj, spd=3.0):
    new_traj = RobotTrajectory()
    new_traj = traj

    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)

    points = list(traj.joint_trajectory.points)
    
    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / spd
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * spd
            point.accelerations[j] = point.accelerations[j] * spd * spd

        points[i] = point

    new_traj.joint_trajectory.points = points

    return new_traj
