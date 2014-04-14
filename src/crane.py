#!/usr/bin/env python

# Jon Rovira
# Summer 2013

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy
from std_msgs.msg import UInt16

####################
# RETHINK IMPORTS: #
####################
import baxter_interface

###############
# NU IMPORTS: #
###############
from trajectory import Trajectory
from vector_operations import (make_vector_from_POINTS,
	                           angle_between_vectors,
	                           vector_projection_onto_plane,
	                           shortest_vector_from_point_to_vector)

##################
# OTHER IMPORTS: #
##################
import operator
import math

###################
# MOVEIT IMPORTS: #
###################
import moveit_commander
import moveit_msgs.msg

from trajectory_speed_up import traj_speed_up

class Crane():
    """
	Crane control class

	This class allows Baxter to mimic an inidividual's left arm with Baxter's
	right arm and grants the individual control of Baxter's gripper. The Crane
	obtains tracking data by subscribing to the /skeleton messages published
	by the skeletontracker_nu script.
	"""

    def __init__(self):
        """
		Crane constructor
		"""
        self.arm = baxter_interface.Limb('right')
        self.gripper = baxter_interface.Gripper('right')
		# self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
		# self.pub_rate.publish(500)
        self.neutral_position = dict(zip(self.arm.joint_names(),
                                         [0.00, 0.00, 1.57, 0.00, 0.00, 0.00, 0.00]))
        self.gripper_state = True
        self.gripper_state_timer = 0
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")

    def set_neutral(self):
        """
		Moves Baxter's arm to neutral position
		"""
		# self.arm.move_to_joint_positions(self.neutral_position)
        self.right_arm.set_joint_value_target(self.neutral_position)
        self.right_arm.go(wait=False)

    def is_neutral(self):
        """
		Checks if Baxter has reached neutral position
		"""
        errors = map(operator.sub, self.arm.joint_angles().values(),
                     self.neutral_position.values())
        for error in errors:
            if math.fabs(error) > 0.06:
                return False
            return True

    def move(self, left_shoulder, left_elbow, left_hand, right_shoulder, right_elbow, right_hand):
        """
		Moves the crane arm and gripper based on human skeleton positions
		"""
        angles = []
        if self.human_to_baxter(left_shoulder, left_elbow, left_hand,
                                right_shoulder, right_elbow, right_hand, angles):
            self.gripper.close()
        else:
            self.gripper.open()


        r_positions = dict(zip(self.arm.joint_names(),
                               [angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]]))
		# self.arm.set_joint_positions(r_positions)
        self.right_arm.set_joint_value_target(r_positions)
        traj = self.right_arm.plan()
        new_traj = traj_speed_up(traj, spd=3.0)
        self.right_arm.execute(new_traj)

    def human_to_baxter(self, l_sh, l_el, l_ha, r_sh, r_el, r_ha, a):
        """
		Computes angles sent to Baxter's arm, checks if gripper should adjust
		"""
		# Crane Arm
        l_upper_arm = make_vector_from_POINTS(l_sh, l_el)
        l_forearm = make_vector_from_POINTS(l_el, l_ha)
		# S0
        v_xz = vector_projection_onto_plane(l_upper_arm, [0,0,-1], [-1,0,0])
        theta = angle_between_vectors(v_xz, [-1,0,0])
        s0 = theta - math.pi/4
		# S1
        theta = angle_between_vectors(l_upper_arm, v_xz)
        if l_el.y > l_sh.y:
            s1 = theta
        else:
            s1 = -theta
		# E0
        n_upper_arm = shortest_vector_from_point_to_vector(l_ha, l_upper_arm,
                                                           l_forearm, l_sh)
        theta = angle_between_vectors(n_upper_arm, [0,1,0])
        e0 = theta
		# E1
        theta = angle_between_vectors(l_upper_arm, l_forearm)
        e1 = theta
		# W0, W1, and W2
        w0 = -1.57
        w1 = 0.00
        w2 = -0.30
		# Check if angles are valid
        self.check_angles(s0,s1,e0,e1,w0,w1,w2,a)

		# Gripper Control Arm
        r_upper_arm = make_vector_from_POINTS(r_sh, r_el)
        r_forearm = make_vector_from_POINTS(r_el, r_ha)
		# Event
        theta = angle_between_vectors(r_upper_arm, r_forearm)
        if theta > 0.8:
            return True
        return False

    def check_angles(self, s0, s1, e0, e1, w0, w1, w2, angles):

        if -0.25 < s0 and s0 < 1.60:
            angles.append(s0)
        elif -0.25 < s0:
            angles.append(1.60)
        else:
            angles.append(-0.25)

		# s1 assignment in safe range
        if -2.00 < s1 and s1 < 0.90:
            angles.append(s1)
        elif -2.00 < s1:
            angles.append(0.90)
        else:
            angles.append(-2.00)

		# e0 assignment
        angles.append(e0)

		# e1 assignment in safe range
        if 0.10 < e1 and e1 < 2.50:
            angles.append(e1)
        elif 0.10 < e1:
            angles.append(2.50)
        else:
            angles.append(0.10)

		# w0, w1, and w2 assignment
        angles.extend([w0,w1,w2])
