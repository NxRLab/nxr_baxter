#!/usr/bin/env python

################
# ROS IMPORTS: #
################
import roslib
roslib.load_manifest('nxr_baxter')
import rospy

####################
# RETHINK IMPORTS: #
####################
import baxter_interface

###############
# NU IMPORTS: #
###############

##################
# OTHER IMPORTS: #
##################
from subprocess import Popen, PIPE
import signal
import threading

class Processes:
	def __init__(self):
		self.process = Popen(['rostopic', 'hz', 'tf'], stdin=PIPE)
		rospy.sleep(1)
		self.process.send_signal(signal.SIGINT)
		process_thread = threading.Thread(target=self.start_process(), name='proc')
		kill_thread = threading.Thread(target=self.kill_process(), name='kill')
		#process_thread.start()
		#kill_thread.start()

	def start_process(self):
		self.process = Popen(['rostopic', 'hz', 'tf'], stdin=PIPE)

	def kill_process(self):
		rospy.sleep(1)
		self.process.send_signal(signal.SIGINT)



if __name__=='__main__':
	print("\nInitializing node... ")
	rospy.init_node('Head_Stuff')
	print "Getting robot state... "
	rs = baxter_interface.RobotEnable()
	print "Enabling robot... "
	rs.enable()

	Processes()

	rs.disable()

	