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

##################
# OTHER IMPORTS: #
##################
import cv
import cv_bridge

class Grabber():
	"""
	Grabber

	Makes Baxter grab an object from a user and pass the object to the
	other arm
	"""

	def __init__(self):
		"""
		Initializes Baxter's arms and grippers
		"""
		self.left_arm = baxter_interface.limb.Limb("left")
		self.right_arm = baxter_interface.limb.Limb("right")
		self.left_gripper = baxter_interface.Gripper("left")
		self.right_gripper = baxter_interface.Gripper("right")

	def gripper_setup(self):
		self.left_gripper.reset()
		self.left_gripper.reboot()
		self.left_gripper.calibrate()

		self.right_gripper.reset()
		self.right_gripper.reboot()
		self.right_gripper.calibrate()

	def grab_from_user(self):
		angles = [0.50, 0.70, 2.40, 0.80, -1.57, 0.00, 1.70]
		joints = self.right_arm.joint_names()
		position = dict(zip(joints, angles))
		self.right_arm.move_to_joint_positions(position)
		rospy.sleep(3.0)
		self.right_gripper.close()
		rospy.sleep(1.0)

	def pass_to_left(self):
		joints_l = self.left_arm.joint_names()
		joints_r = self.right_arm.joint_names()
		angles_l = [0.30, 0.55, -1.55, 1.85, 1.57, 0.00, -0.80]
		angles_r = [0.10, 0.65, 1.70, 1.60, -1.65, 0.00, 0.95]
		position_l = dict(zip(joints_l, angles_l))
		position_r = dict(zip(joints_r, angles_r))

		self.left_arm.move_to_joint_positions(position_l)
		self.right_arm.move_to_joint_positions(position_r)
		rospy.sleep(2.0)

		self.left_gripper.close()
		rospy.sleep(1.0)
		self.right_gripper.open()
		self.left_gripper.inc_position(.50)
		rospy.sleep(1.0)

	def put_on_table(self):
		self.left_gripper.close()

		joints_l = self.left_arm.joint_names()
		joints_r = self.right_arm.joint_names()
		angles_l = [0.90, 0.80, -1.55, 1.35, 2.50, 0.00, 0.10]
		angles_r = [0.10, 0.50, 1.70, 1.00, -1.57, 0.00, 1.45]
		position_l = dict(zip(joints_l, angles_l))
		position_r = dict(zip(joints_r, angles_r))

		self.right_arm.move_to_joint_positions(position_r)
		self.left_gripper.inc_position(.80)
		self.left_arm.move_to_joint_positions(position_l)
		rospy.sleep(1.0)

		self.left_gripper.open()
		rospy.sleep(1.0)

	def back_to_neutral(self):
		joints_l = self.left_arm.joint_names()
		joints_r = self.right_arm.joint_names()
		angles_l = [1.50, 0.80, -1.55, 2.20, 2.50, 0.00, 0.10]
		position_l = dict(zip(joints_l, angles_l))
		self.left_arm.move_to_joint_positions(position_l)
		rospy.sleep(1.0)


	def grab(self):
		self.gripper_setup()
		self.grab_from_user()
		self.pass_to_left()
		self.put_on_table()
		self.back_to_neutral()


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

	grabber = Grabber()
	grabber.grab()

	print "Disabling robot..."
	rs.disable()
	print "Done."