#!/usr/bin/env python

# Jon Rovira
# Summer 2013

################
# ROS IMPORTS: #
################
import rospy
from std_msgs.msg import UInt16

####################
# RETHINK IMPORTS: #
####################
import baxter_interface
import baxter_dataflow

###############
# NU IMPORTS: #
###############
from trajectory import Trajectory
from vector_operations import (make_vector_from_POINTS,
	                           angle_between_vectors,
	                           vector_projection_onto_plane,
	                           shortest_vector_from_point_to_vector)

###################
# MOVEIT IMPORTS: #
###################
import moveit_commander
import moveit_msgs.msg

##################
# OTHER IMPORTS: #
##################
import os
import traceback
import threading
import Queue
import math
import operator
from pprint import pprint

from trajectory_speed_up import traj_speed_up



class Mime():
    """
    Mime control class

    This class controls Baxter by making him mimic an individual's
    movements that are being tracked by an Asus Xtion sensor. The Mime
    obtains the tracking data by subscribing to the /skeleton messsages
    published by the skeletontracker_nu script.
    """

    def __init__(self,):
        """
        Mime constructor
        """
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')

        # self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
        # self.pub_rate.publish(500)
        self.neutral_position_l = dict(zip(self.left_arm.joint_names(),
                                           [0.60, 0.40, -2.90, 1.70, 1.57, 0.0, 0.0]))
        self.neutral_position_r = dict(zip(self.right_arm.joint_names(),
                                           [-0.60, 0.40, 2.90, 1.70, -1.57, 0.0, 0.0]))
        self.both_arms = moveit_commander.MoveGroupCommander("both_arms")
        self.both_arms.set_planning_time(0.01)

    def set_neutral(self):
        """
        Moves Baxter's arms to neutral positions
        """
        self.both_arms.stop()
        self.both_arms.set_joint_value_target(dict(self.neutral_position_l,
                                                   **self.neutral_position_r))
        traj = self.both_arms.plan()
        new_traj = traj_speed_up(traj, spd=3.0)
        self.moveit_both_arms.execute(new_traj)

    def is_neutral(self):
        """
        Checks if Baxter has reached neutral positions
        """
        errors_l = map(operator.sub, self.left_arm.joint_angles().values(),
                       self.neutral_position_l.values())
        errors_r = map(operator.sub, self.right_arm.joint_angles().values(),
                       self.neutral_position_r.values())
        errors_l.extend(errors_r)
        for error in errors_l:
            if math.fabs(error) > 0.06:
                return False
        return True

    def move(self, left_shoulder, left_elbow, left_hand,
             right_shoulder, right_elbow, right_hand):
        """
        Creates and sends threads for Baxter's arm movement
        """
        angles = {'left': [], 'right': []}
        self.human_to_baxter(left_shoulder, left_elbow, left_hand,
                             right_shoulder, right_elbow, right_hand, angles)
        position = angles['left']
        l_positions = dict(zip(self.left_arm.joint_names(),
                               [position[0], position[1], position[2],
                                    position[3], position[4], position[5],
                                    position[6]]))
        position = angles['right']
        r_positions = dict(zip(self.right_arm.joint_names(),
                               [position[0], position[1], position[2],
                                    position[3], position[4], position[5],
                                    position[6]]))
        self.both_arms.stop()
        self.both_arms.set_joint_value_target(dict(l_positions, **r_positions))
        traj = self.both_arms.plan()
        new_traj = traj_speed_up(traj, spd=3.0)
        self.both_arms.execute(new_traj)

    def human_to_baxter(self, l_sh, l_el, l_ha, r_sh, r_el, r_ha, a):
        """
        Determines how to move Baxter's limbs in order to mimic user
        """

        for arm in ['left', 'right']:
            if arm=='left':
                sh = l_sh
                el = l_el
                ha = l_ha
            elif arm=='right':
                sh = r_sh
                el = r_el
                ha = r_ha

            upper_arm = make_vector_from_POINTS(sh, el)
            forearm = make_vector_from_POINTS(el, ha)

            # E0 computations
            n_upper_arm = shortest_vector_from_point_to_vector(ha, upper_arm,
                                                               forearm, sh)
            theta = angle_between_vectors(n_upper_arm, [0,1,0])
            e0 = theta
            # E1 computations
            theta = angle_between_vectors(upper_arm, forearm)
            e1 = theta
            # W0, W1, and W2 computations
            w0 = 0.00
            w1 = 0.00
            w2 = 0.00

            if arm=='left':
                # S0 computations
                v_xz = vector_projection_onto_plane(upper_arm, [0,0,-1], [-1,0,0])
                theta = angle_between_vectors(v_xz, [-1,0,0])
                s0 = theta - math.pi/4
                # S1 computations
                theta = angle_between_vectors(upper_arm, v_xz)
                if el.y > sh.y:
                    s1 = theta
                else:
                    s1 = -theta
                angles = a['right']
                # s0 assignment in safe range
                if -0.25 < s0 and s0 < 1.60:
                    angles.append(s0)
                elif -0.25 < s0:
                    angles.append(1.60)
                else:
                    angles.append(-0.25)


            elif arm=='right':
                # S0 computations
                v_xz = vector_projection_onto_plane(upper_arm, [0,0,-1], [1,0,0])
                theta = angle_between_vectors(v_xz, [-1,0,0])
                s0 = theta - 3*math.pi/4
                # S1 computations
                theta = angle_between_vectors(upper_arm, v_xz)
                if el.y > sh.y:
                    s1 = theta
                else:
                    s1 = -theta
                e0 = -e0
                w0 = -w0
                w2 = -w2
                angles = a['left']
                # s0 assignment in safe range
                if -1.60 < s0 and s0 < 0.25:
                    angles.append(s0)
                elif -1.60 < s0:
                    angles.append(0.25)
                else:
                    angles.append(-1.60)

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

        return
