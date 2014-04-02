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

from nxr_baxter_msgs.msg import MetaMode

from nxr_baxter_msgs.srv import *

# PYTHON IMPORTS
import os
import signal
import subprocess

# Function taken from Jarvis/Jon's script to kill all child processes
def terminate_process_and_children(p):
    rospy.logdebug("Calling terminate_process_and_children")
    rospy.logdebug("Terminating process %d", p.pid)
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
        os.kill(int(pid_str), signal.SIGINT)
    p.terminate()

class Heartbeat_Monitor:
    def __init__(self):
        rospy.logdebug("Calling Heartbeat_Monitor.__init__()")
        # For calculating skeleton heartbeat
        # Average it every 5 seconds.
        self.heartbeat_period = 5.0
        # Initialize count
        self._heartbeat_count = 0

        self.n_moving_avg_filt = 12
        self.max_allowable_frequency = 24.0
        self.freq_filter_list = Heartbeat_List(self.n_moving_avg_filt)

        # Time to wait to start (1 min)
        self.delay_start = 60

        self.kill_count = 0

        # Make a subscriber to call a function to track the heartbeat of the
        # skeleton tracker
        rospy.Subscriber("tracker_heartbeat", Empty, self.empty_skel_callback)
        rospy.Subscriber("meta_mode", MetaMode, self.meta_mode_callback)

        #Start the delay, and launch the regular timer after the delay_start
        self.start_delay_timer()

        # We will launch openni and start the skeleton tracker here.
        self.openni_proc = None
        self.skel_tracker_proc = None
        self.launch_processes()


    #Start the delay timer
    def start_delay_timer(self):
        rospy.logdebug("Calling start_delay_timer")
        #clear the Heartbeat List
        self.freq_filter_list.clear()
        self.delay_timer = rospy.Timer(rospy.Duration(self.delay_start), self.start_main_timer, oneshot=True)

    #Start the main timer after the delay timer has been launched
    def start_main_timer(self, event=None):
        rospy.logdebug("Calling start_main_timer")
        self.main_timer = rospy.Timer(rospy.Duration(self.heartbeat_period),
                                          self.heartbeat_timer_callback)

    #Callback for the heartbeat. Updates the heartbeat count.
    #There will be a separate timer to calculate the actual frequency
    def empty_skel_callback(self, event):
        rospy.logdebug("Calling empty_skel_callback()")
        self._heartbeat_count += 1

    # Callback for heartbeat timer calculation. Gets the current count and will
    # calculate the average. Keeps an updated list of the past n_moving_avg_filt
    # frequencies and if its less than 25, shutdown and restart the processes
    def heartbeat_timer_callback(self, event):
        rospy.logdebug("Calling heartbeat_timer_callback")
        # Calculate frequency
        ht_bt_freq = self._heartbeat_count/self.heartbeat_period
        self.freq_filter_list.push(ht_bt_freq)
        #Reset count
        self._heartbeat_count = 0
        rospy.loginfo("Averaged tracker frequency: %d", self.freq_filter_list.sum/self.n_moving_avg_filt)
        if self.freq_filter_list.sum/self.n_moving_avg_filt < self.max_allowable_frequency:
            # self.shutdown_and_restart()
            # Now send a service request to change the mode so we can restart
            # kinect and stuff, then our callback on that will call
            # shutdown_and_restart
            rospy.wait_for_service('change_meta_mode')
            try:
                change_mode = rospy.ServiceProxy('change_meta_mode', ChangeMetaMode)
                # change_srv = ChangeMetaModeRequest()
                # change_srv.mode = change_srv.RESTART_KINECT
                # change_success = change_mode(change_srv)
                change_resp = change_mode(ChangeMetaModeRequest.RESTART_KINECT)
                if not change_resp.error:
                    rospy.logerr("Tried to restart Kinect, but mode change request failed.")
                else:
                    rospy.logdebug("Kinect shutdown meta mode change succeeded.")
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s",e)

    def meta_mode_callback(self, data):
        rospy.logdebug("Calling meta_mode_callback")
        if data.mode == data.RESTART_KINECT:
            self.shutdown_and_restart()
            #Once restarted, tell it we can go back to idle enabled mode
            rospy.wait_for_service('change_meta_mode')
            try:
                change_mode = rospy.ServiceProxy('change_meta_mode', ChangeMetaMode)
                # change_srv = ChangeMetaModeRequest()
                # change_srv.mode = change_srv.IDLE_ENABLED
                # change_success = change_mode(change_srv)
                change_resp = change_mode(ChangeMetaModeResponse.IDLE_ENABLED)
                if not change_resp.error:
                    rospy.logerr("Tried to go back to idle mode, failed!")
                else:
                    rospy.logdebug("Back to idle mode meta mode change succeeded.")
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s",e)

    #This function is called when the frequency gets bad
    def shutdown_and_restart(self):
        rospy.logdebug("Calling shutdown_and_restart")
        self.kill_count += 1
        rospy.loginfo("Shutdown count: %d", self.kill_count)
        if self.main_timer != None:
            rospy.logdebug("Shutting down main timer")
            self.main_timer.shutdown()
        rospy.loginfo("Killing nodelet...")
        p2 = subprocess.Popen("rosnode kill /camera_nodelet_manager", shell=True, stdout=subprocess.PIPE)
        rospy.sleep(2.0)
        rospy.loginfo("Killing skeleton tracker...")
        terminate_process_and_children(self.skel_tracker_proc)
        rospy.sleep(2.0)
        rospy.loginfo("Killing openni processes...")
        terminate_process_and_children(self.openni_proc)
        
        #USB restart
        rospy.loginfo("Restarting usb...")
        cmd = "/home/adam-baxter/adam_groovy_ws/src/nxr_baxter_demo_package/src/restart_usb.sh"
        subprocess.call(cmd, shell=True)

        rospy.sleep(10.0)
        #restart processes
        self.launch_processes()
        self.start_delay_timer()

    def launch_processes(self):
        rospy.logdebug("Calling launch_processes")
        if self.openni_proc == None or self.openni_proc.poll() != None:
            rospy.loginfo("Launching openni processes...")
            cmd = 'roslaunch openni_launch openni.launch'
            self.openni_proc = subprocess.Popen(cmd,shell=True)
        else:
            rospy.logwarn("Trying to start openni thread while it is already running.")

        if self.skel_tracker_proc == None or self.skel_tracker_proc.poll() != None:
            rospy.loginfo("Launching skeleton tracker...")
            cmd = 'rosrun skeletontracker_nu skeletontracker'
            self.skel_tracker_proc = subprocess.Popen(cmd,shell=True)
        else:
            rospy.logwarn("Trying to start skeleton tracker thread while it is already running.")


class Heartbeat_List:
    """
    A list for tracking the frequency for the heartbeat. Pushing to this list
    will remove the oldest element and replace it with the pushed element. You
    can also ask for the current sum of all elements in the list. The list is
    initialized to zero with a length given when initializing, defaults to 1.
    """

    def __init__(self,length=1):
        rospy.logdebug("Calling Heartbeat_List.__init__()")
        self._list = [0]*length
        self.sum = 0.0
        self._oldest_index = 0
        self._max_index = length - 1
        self.num_pushed = 0

    def push(self, val):
        rospy.logdebug("Calling Heartbeat_List.push(val)")
        if self.num_pushed <= self._max_index:
            self.num_pushed += 1
        else:
            self.sum -= self._list[self._oldest_index]
        self.sum += val
        self._list[self._oldest_index] = val
        self._oldest_index += 1
        if self._oldest_index > self._max_index:
            self._oldest_index = 0
    
    def clear(self):
        rospy.logdebug("Calling Heartbeat_List.clear()")
        self._list = [0]*(self._max_index + 1)
        self.sum = 0.0
        self._oldest_index = 0
        self.num_pushed = 0

    def get_average(self):
        return self.sum/self.num_pushed


if __name__=='__main__':
    rospy.loginfo("Starting Heartbeat Tracker Node...")
    rospy.init_node('Heartbeat_Tracker', log_level=rospy.INFO)
    rospy.logdebug("node starting")
    hm = Heartbeat_Monitor()

    # rospy.sleep(60)
    # rospy.loginfo("Attempting to kill node...")
#    hm.shutdown_and_restart()
    
    rospy.spin()

    rospy.loginfo("Heartbeat Tracker shutting down.")
