#!/usr/bin/env python

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy
from std_msgs.msg import UInt16

####################
# NON ROS IMPORTS: #
####################
import os
import traceback
import threading
import Queue
import baxter_interface
import iodevices
import dataflow
import baxter_msgs


class Clapper():
	def __init__(self):
		self.left_arm = baxter_interface.Limb('left')
		self.right_arm = baxter_interface.Limb('right')
		self.pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
		self.pub_rate.publish(500)
		self.result = [False, '']

	def move_thread(self, limb, angles, queue, timeout=15.0):
		try:	
			limb.move_to_joint_positions(angles, timeout)
			queue.put(None)
	
		except Exception, exception:
			queue.put(traceback.format_exc())
			queue.put(exception)
	
	def open_arms(self):
		angles_l = [ 0.4, 0.2, -1.0, 0.4, 0.0, 0.4, 0.0]
		angles_r = [-0.4, 0.2,  1.0, 0.4, 0.0, 0.4, 0.0]
	
		left_queue = Queue.Queue()
		right_queue = Queue.Queue()

		left_thread = threading.Thread(
	    	target=self.move_thread,
            args=(self.left_arm,
                  dict(zip(self.left_arm.joint_names(), angles_l)),
                  left_queue)
	    	)
		right_thread = threading.Thread(
	    	target=self.move_thread,
	    	args=(self.right_arm,
                  dict(zip(self.right_arm.joint_names(), angles_r)),
                  right_queue)
            )

		left_thread.daemon = True
		right_thread.daemon = True
		left_thread.start()
		right_thread.start()
		dataflow.wait_for(
	    	lambda: \
	    	not (left_thread.is_alive() or right_thread.is_alive()),
	    	timeout=20.0,
	    	rate=10,
	    	)
		left_thread.join()
		right_thread.join()
		result = left_queue.get()
		if not result == None:
			raise left_queue.get()
		result = right_queue.get()
		if not result == None:
			raise right_queue.get()
		rospy.sleep(1.0)
	
	def close_arms(self):
		angles_l = [-0.6, 0.2, -1.8, 0.4, 0.0, 0.4,  0.0]
		angles_r = [ 0.6, 0.3,  1.8, 0.4, 0.0, 0.4, -0.3]

		left_queue = Queue.Queue()
		right_queue = Queue.Queue()

		left_thread = threading.Thread(
			target=self.move_thread,
			args=(self.left_arm,
                  dict(zip(self.left_arm.joint_names(), angles_l)),
                  left_queue))
		right_thread = threading.Thread(
			target=self.move_thread,
	    	args=(self.right_arm,
				  dict(zip(self.right_arm.joint_names(), angles_r)),
				  		   right_queue))

		left_thread.daemon = True
		right_thread.daemon = True
		left_thread.start()
		right_thread.start()
		dataflow.wait_for(
			lambda: \
			not (left_thread.is_alive() or right_thread.is_alive()),
			timeout=20.0,
			rate=10,
			)
		left_thread.join()
		right_thread.join()
		result = left_queue.get()
		if not result == None:
			raise left_queue.get()
		result = right_queue.get()
		if not result == None:
			raise right_queue.get()
		rospy.sleep(1.0)
	
	def clap(self):
		self.open_arms()
		self.close_arms()

