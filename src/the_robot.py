#!/usr/bin/env python

# Jon Rovira
# Fall 2013

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy
import sensor_msgs.msg

####################
# RETHINK IMPORTS: #
####################
import baxter_interface
import dataflow

###############
# NU IMPORTS: #
###############
from trajectory import Trajectory

##################
# OTHER IMPORTS: #
##################
import threading
import Queue
import traceback
import cv
import cv_bridge


class The_Robot():
	"""
	The Robot

	Makes Baxter do the robot dance.
	"""

	def __init__(self):
		"""
		Initializes Baxter's arms
		"""
		self.left_arm = baxter_interface.limb.Limb("left")
		self.right_arm = baxter_interface.limb.Limb("right")
		self.moves = ['neutral', 'first', 'second', 'third', 'fourth']

	def move_thread(self, limb, position, queue, timeout=15.0):
		"""
		Adds the points sent by threads to the trajectory path
		"""
		try:
			trajectory = Trajectory(limb)
			trajectory.add_point(position, 2.0)
			trajectory.start()
			queue.put(None)
		except Exception, exception:
			queue.put(traceback.format_exc())
			queue.put(exception)

	def all_moves(self, step):
		"""
		Defines different positions for Baxter's Robot dance
		"""
		moves = {'neutral_l': [0.60, 1.00, -1.50, 1.70, 1.57, 0.00, 0.00],
		         'neutral_r': [-0.60, 1.00, 1.50, 1.70, -1.57, 0.00, 0.00],
		         'first_l': [0.30, 0.70, -1.50, 1.90, 1.57, 0.00, 0.00],
		         'first_r': [-0.90, 1.00, 1.50, 1.70, -1.57, 0.00, 0.00],
		         'second_l': [0.90, 1.00, -1.50, 1.70, 1.57, 0.00, 0.00],
		         'second_r': [0.80, 0.70, 2.80, 1.90, -1.57, 0.00, 0.00],
		         'third_l': [0.90, 1.00, -1.50, 1.70, 1.57, 0.00, 0.00],
		         'third_r': [-0.10, 0.70, 1.50, 1.90, -1.57, 0.00, 0.00],
		         'fourth_l': [0.10, 0.30, -1.80, 1.90, 1.57, 0.00, 0.00],
		         'fourth_r': [-1.20, 1.00, 1.10, 2.00, -1.57, 0.00, 0.00]
		         }


		return moves[step]

	def set_neutral(self):
		"""
		Sets Baxter's arms to a neutral position in order to 
		start dancing
		"""
		print "Moving to neutral pose..."
		angles_l = dict(zip(self.left_arm.joint_names(),
		                    self.all_moves('neutral_l')))
		angles_r = dict(zip(self.right_arm.joint_names(),
			                self.all_moves('neutral_r')))

		self.left_arm.move_to_joint_positions(angles_l)
		self.right_arm.move_to_joint_positions(angles_r)

	def dance(self):
		self.set_neutral()

		done = False
		while not done and not rospy.is_shutdown():
			for move in self.moves:
				angles = {'left': self.all_moves(move + '_l'),
				          'right': self.all_moves(move + '_r')
				          }
				print angles['left']
				print angles['right']

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
				dataflow.wait_for(lambda: \
			              not (left_thread.is_alive() or right_thread.is_alive()),
			              timeout=20.0,
			              rate=10)
				left_thread.join()
				right_thread.join()
				rospy.sleep(2.0)





if __name__ == '__main__':
	print "Initializing node..."
	rospy.init_node("The_Robot")
	print "Getting robot state..."
	rs = baxter_interface.RobotEnable()

	img = cv.LoadImage("/home/nxr-baxter/catkin_ws/src/nxr_baxter/images/nxr.jpg")
	msg = cv_bridge.CvBridge().cv_to_imgmsg(img)
	pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
	pub.publish(msg)

	print "Enabling robot..."
	rs.enable()

	the_robot = The_Robot()
	the_robot.dance()

	print "Disabling robot..."
	rs.disable()
	print "Done dancing."


	