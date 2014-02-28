#!/usr/bin/env python

################
# ROS IMPORTS: #
################
import rospy
import actionlib
from std_msgs.msg import UInt16
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import (JointTrajectory,
	                             JointTrajectoryPoint)

####################
# RETHINK IMPORTS: #
####################
import baxter_dataflow
import baxter_interface

############################
# OTHER IMPORTS: #
############################
from copy import copy
import os

class Trajectory(object):
	def __init__(self, limb):
		ns = 'robot/limb/' + limb + '/'
		self._client = actionlib.SimpleActionClient(
			ns + "follow_joint_trajectory",
		    FollowJointTrajectoryAction,
		)
		self._goal = FollowJointTrajectoryGoal()
		server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
		if not server_up:
			rospy.logerr("Timed out waiting for Joint Trajectory"
				         " Action Server to connect. Start the action server"
				         " before running example.")
			rospy.signal_shutdown("Timed out waiting for Action Server")
			sys.exit(1)
		self.clear(limb)

	def add_point(self, positions, time):
		point = JointTrajectoryPoint()
		point.positions = copy(positions)
		point.time_from_start = rospy.Duration(time)
		# Dumb "_.goal"
		self._goal.trajectory.points.append(point)

	def start(self):
		self._goal.trajectory.header.stamp = rospy.Time.now()
		self._client.send_goal(self._goal)

	def stop(self):
		self._client.cancel_goal()

	def wait(self, timeout=15.0):
		self.client.wait_for_result(timeout=rospy.Duration(timeout))

	def result(self):
		return self._client.get_result()

	def clear(self, limb):
		self._goal = FollowJointTrajectoryGoal()
		self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
			['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]



