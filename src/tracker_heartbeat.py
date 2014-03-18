#!/usr/bin/env python

# Adam Barber
# March 2014

# This script monitors the heartbeat message of the skeleton tracker and makes
# sure it hasn't slowed down too much. This is a full node that runs separately
# from baxter_controller.py and can kill it on its own. Or at least will
# eventually.

# ROS IMPORTS
import rospy
from std_msgs.msg import Empty

# PYTHON IMPORTS
import os
import signal
import subprocess

class Heartbeat_Monitor:
    def __init__(self):
        # For calculating skeleton heartbeat
        # Average it every 5 seconds.
        self.heartbeat_period = 5.0
        # Initialize count
        self._heartbeat_count = 0

        self.n_moving_avg_filt = 12
        self.max_allowable_frequency = 25.0
        self.freq_filter_list = Heartbeat_List(self.n_moving_avg_filt)

        # Time to wait to start (1 min)
        self.delay_start = 60

        # Make a subscriber to call a function to track the heartbeat of the
        # skeleton tracker
        rospy.Subscriber("tracker_heartbeat", Empty, self.heartbeatCallback)

        #Start the delay, and launch the regular timer after the delay_start
        rospy.Timer(rospy.Duration(self.delay_start),
                    lambda event: rospy.Timer(rospy.Duration(self.heartbeat_period),
                                              self.heartbeat_timer_callback),
                                              oneshot=True)

        # We will be calling the roslaunch file from here for the skeleton tracker
        # Command is "roslaunch skeletontracker_nu nu_skeletontracker.launch"
        # Or instead... "rosrun skeletontracker_nu skeletontracker"
        # And we can run openni stuff in another thread as:
        # "roslaunch openni_launch openni.launch"
        cmd = 'roslaunch openni_launch openni.launch'
        self.openni_proc = subprocess.Popen(cmd,shell=True, stdout = subprocess.PIPE)
        cmd = 'rosrun skeletontracker_nu skeletontracker'
        self.skel_tracker_proc = subprocess.Popen(cmd,shell=True, stdout = subprocess.PIPE)

    #Callback for the heartbeat. Updates the heartbeat count.
    #There will be a separate timer to calculate the actual frequency
    def heartbeatCallback(self, event):
        self._heartbeat_count += 1

    # Callback for heartbeat timer calculation. Gets the current count and will
    # calculate the average. Keeps an updated list of the past n_moving_avg_filt
    # frequencies and if its less than 25, shutdown and restart the processes
    def heartbeat_timer_callback(self, event):
        # Calculate frequency
        ht_bt_freq = self._heartbeat_count/self.heartbeat_period
        self.freq_filter_list.push(ht_bt_freq)
        #Reset count
        self._heartbeat_count = 0
        print self.freq_filter_list.sum/self.n_moving_avg_filt
        if self.freq_filter_list.sum/self.n_moving_avg_filt < self.max_allowable_frequency:
            self.shutdown_and_restart()

    #This function is called when the frequency gets bad
    def shutdown_and_restart(self):
        print "Attempting to send SIGINT to skel tracker"
        self.skel_tracker_proc.send_signal(signal.SIGINT)
        print "SIGINT sent to tracker, sending SIGKILL to openni"
        self.openni_proc.kill()
        # #First kill /camera_nodelet_manager
        
        # #When do we call USB restart?
        # #Wait two seconds
        # #Get pid for skeletontrackernu
        # pid_list = subprocess.check_output(["pgrep","skeletontracker"])
        # #kill the first pid, split by '\n'
        # print pid_list
        # pid = int(pid_list.split('\n')[0])
        # print pid
        # #Run something like
        # #os.kill(pid,sig) for skeletontrackernu, it will startup on its own
        # os.kill(pid, signal.SIGKILL)
        # print("Sent SIGKILL")


class Heartbeat_List:
    """
    A list for tracking the frequency for the heartbeat. Pushing to this list
    will remove the oldest element and replace it with the pushed element. You
    can also ask for the current sum of all elements in the list. The list is
    initialized to zero with a length given when initializing, defaults to 1.
    """

    def __init__(self,length=1):
        self._list = [0]*length
        self.sum = 0.0
        self._oldest_index = 0
        self._max_index = length - 1

    def push(self, val):
        self.sum -= self._list[self._oldest_index]
        self.sum += val
        self._list[self._oldest_index] = val
        self._oldest_index += 1
        if self._oldest_index > self._max_index:
            self._oldest_index = 0


if __name__=='__main__':
    print("\nInitializing Heartbeat Tracker node... ")
    rospy.init_node('Heartbeat_Tracker', log_level=rospy.INFO)
    rospy.logdebug("node starting")
    hm = Heartbeat_Monitor()
    rospy.sleep(30)
    print("Attempting to kill node...")
    hm.shutdown_and_restart()
    
    rospy.spin()

    print("\nHeartbeat Tracker shutting down.")
