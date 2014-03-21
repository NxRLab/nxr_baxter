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



class Mime():
	"""
	Mime control class

	This class controls Baxter by making him mimic an individual's
	movements that are being tracked by an Asus Xtion sensor. The Mime
	obtains the tracking data by subscribing to the /skeleton messsages
	published by the skeletontracker_nu script.
	"""

	def __init__(self):
		"""
		Mime constructor
		"""
		self.left_arm = baxter_interface.Limb('left')
		self.right_arm = baxter_interface.Limb('right')

		self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
		self.pub_rate.publish(500)
		self.neutral_position_l = dict(zip(self.left_arm.joint_names(),
			                               [0.60, 0.40, -2.90, 1.70, 1.57, 0.0, 0.0]))
		self.neutral_position_r = dict(zip(self.right_arm.joint_names(),
			                               [-0.60, 0.40, 2.90, 1.70, -1.57, 0.0, 0.0]))

	def set_neutral(self):
		"""
		Moves Baxter's arms to neutral positions
		"""
		self.left_arm.move_to_joint_positions(self.neutral_position_l)
		self.right_arm.move_to_joint_positions(self.neutral_position_r)

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

	def move_thread(self, limb, position, queue, timeout=15.0):
		"""
		Adds the points sent by threads to the trajectory path
		"""
		if limb == 'left':
			l_positions = dict(zip(self.left_arm.joint_names(),
				                   [position[0], position[1], position[2], position[3], position[4], position[5], position[6]]))
			self.left_arm.set_joint_positions(l_positions)
		elif limb == 'right':
			r_positions = dict(zip(self.right_arm.joint_names(),
				                   [position[0], position[1], position[2], position[3], position[4], position[5], position[6]]))
			self.right_arm.set_joint_positions(r_positions)

	def move(self, left_shoulder, left_elbow, left_hand,
		     right_shoulder, right_elbow, right_hand):
		"""
		Creates and sends threads for Baxter's arm movement
		"""
		# Collects angles from each controller
		angles = {'left': [], 'right': []}
		self.human_to_baxter(left_shoulder, left_elbow, left_hand,
		                     right_shoulder, right_elbow, right_hand, angles)
		
		# Creates threads for each of Baxter's arms
		left_queue = Queue.Queue()
		right_queue = Queue.Queue()
		left_thread = threading.Thread(target=self.move_thread,
			                           args=('left', angles['left'], left_queue))
		right_thread = threading.Thread(target=self.move_thread,
			                            args=('right', angles['right'], right_queue))
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

	def human_to_baxter(self, l_sh, l_el, l_ha, r_sh, r_el, r_ha, a):
		"""
		Determines how to move Baxter's limbs in order to mimic user
		"""
		# Max Joint Range
		#     (  s0,      s1,    e0,     e1,     w0,     w1,      w2)
    	#     ( 1.701,  1.047,  3.054,  2.618,  3.059,  2.094,  3.059)
    	# Min Joint Range
    	#     (  s0,     s1,     e0,     e1,      w0,    w1,     w2) 
    	#     (-1.701, -2.147, -3.054, -0.050, -3.059, -1.571, -3.059)

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
