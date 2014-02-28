#!/usr/bin/env python

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy

###############
# NU IMPORTS: #
###############
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint
from trajectory_class import Trajectory
from vector_operations import inv_rotate

class Skeleton_Tester():
	def __init__(self):
		self.userid_chosen = False
		self.count = 1
		rospy.Subscriber("skeletons", Skeletons, self.skeletonCallback)

	def choose_user(self, skeleton):
	    self.main_userid = skeleton.userid
	    self.userid_chosen = True
	    print "Main user chosen.\nUser %s, please proceed.\n" % str(self.main_userid)

	def skeletonCallback(self, data):
		skel = data.skeletons[0]
		self.count += 1

		# Chooses and sticks to one main user throughout
		if self.userid_chosen == False:
			self.choose_user(skel)
	
		elif (skel.userid == self.main_userid) and (self.count % 5 = 0):
			h = skel.left_hand.transform.translation
			#v_prime = inv_rotate(q, [1,0,0])
			print "x: " + str(h.x)
			print "y: " + str(h.y)
			print "z: " + str(h.z) + "\n"	


if __name__=='__main__':
	rospy.init_node('Skeleton_Tester')
	Skeleton_Tester()
	rospy.spin()