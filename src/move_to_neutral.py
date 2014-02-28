#!/usr/bin/env python

# Jon Rovira
# Fall 2013

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



class Neutral_Mover():
	"""
	Moves to neutral position

	"""

	def __init__(self):
		self._pub_rate = rospy.Publisher('/robot/joint_state_publish_rate', UInt16)
		self.left_arm = baxter_interface.limb.Limb("left")
		self.right_arm = baxter_interface.limb.Limb("right")

	def set_neutral(self):
		"""
		Sets both arms back into a neutral pose

		"""
		print ("Moving to neutral pose...")
		self.left_arm.move_to_neutral()
		self.right_arm.move_to_neutral()

if __name__ == '__main__':
	print ("Initializing node...")
	rospy.init_node("Neutral_Mover")
	print("Getting robot state...")
	rs = baxter_interface.RobotEnable()
	print("Enabling robot...")
	rs.enable()

	neutral_mover = Neutral_Mover()
	neutral_mover.set_neutral()

	print("Disabling robot...")
	rs.disable()
	print("done.")
