#!/usr/bin/env python

# Adam Barber
# March 2014

# This script monitors the heartbeat message of the skeleton tracker and makes
# sure it hasn't slowed down too much. This is a full node that runs separately
# from baxter_controller.py and can kill it on its own. Or at least will
# eventually.

# ROS IMPORTS
import rospy
import roslib
import rosnode
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import EmptyResponse
from std_srvs.srv import EmptyRequest
from nxr_baxter_msgs.msg import MetaMode
from nxr_baxter_msgs.srv import *
from time import strftime

# PYTHON IMPORTS
import os
import signal
import subprocess
import threading
from collections import deque
import numpy as np

# LOCAL IMPORTS
from process_manager import terminate_process_and_children

# GLOBAL CONSTANTS
MAX_AVG_PERIOD = 1/24.0 # max average period of heartbeat
NUM_AVERAGES = 30 # number of samples to average over
RESTART_TIMEOUT = 60 # how long before restarting computer?
    
class HeartbeatMonitor:
    def __init__(self):
        rospy.logdebug("Calling HeartbeatMonitor.__init__()")
        with open(os.path.join(os.path.expanduser("~"), "shutdown_startup.log"), "a") as fi:
            to_print = "[" + strftime("%Y-%m-%d %H:%M:%S") + "] Starting up\n"
            fi.write(to_print)

        # init process vars:
        self.openni_proc = None
        self.tracker_proc = None
        self.first_flag = True
        self.heartbeat_time = rospy.Time.now()
        self.skelcb_count = 0

        # create deque for holding periods of tracker_heartbeat
        self.heartbeat_samples = deque(maxlen=NUM_AVERAGES)
        
        # create subscribers:
        self.heartbeat_sub = rospy.Subscriber("tracker_heartbeat", Empty,
                                              self.emptycb)
        # self.metamode_sub = rospy.Subscriber("meta_mode", MetaMode,
        #                                      self.meta_mode_callback)

        # create service clients:
        rospy.loginfo("Waiting for change_meta_mode service...")
        rospy.wait_for_service("change_meta_mode")
        self.change_mode = rospy.ServiceProxy("change_meta_mode", ChangeMetaMode)

        # create services for starting and stopping Kinect:
        self.start_kin_service = rospy.Service('start_kinect', EmptySrv, self.start_procs)
        self.stop_kin_service = rospy.Service('stop_kinect', EmptySrv, self.stop_procs)

        # create a timer for polling skeleton tracker:
        self.tracker_timer = rospy.Timer(rospy.Duration(1.0), self.checkskel_cb)
        self.check_count = 0

        # start 
        self.start_procs(EmptyRequest())
        return

        
    def start_procs(self, req):
        rospy.logdebug("/start_kinect service called")
        if self.openni_proc == None:
            rospy.loginfo("Launching openni processes...")
            cmd = 'roslaunch openni_launch openni.launch'
            self.openni_proc = subprocess.Popen(cmd, shell=True)
        else:
            rospy.logwarn("Trying to start openni thread while it is already running.")

        if self.tracker_proc == None:
            rospy.loginfo("Launching skeleton tracker...")
            cmd = 'rosrun skeletontracker_nu skeletontracker'
            self.tracker_proc = subprocess.Popen(cmd,shell=True)
        else:
            rospy.logwarn("Trying to start skeleton tracker thread while it is already running.")

        # reset vars:
        self.first_flag = True    
        return EmptyResponse()



    def stop_procs(self, req):
        rospy.loginfo("/stop_kinect service called")
        if self.openni_proc != None or self.tracker_proc != None:
            rospy.loginfo("processes not None... attempting to shutdown")

            # Kill openni_launch
            rospy.loginfo("Killing openni processes...")
            try:
                terminate_process_and_children(self.openni_proc)
            except:
                rospy.logwarn("Exception while killing drivers")

            # Kill skeleton tracker
            rospy.loginfo("Killing skeleton tracker processes...")
            try:
                terminate_process_and_children(self.tracker_proc)
            except:
                rospy.logwarn("Exception while killing tracker")
            self.openni_proc = None
            self.tracker_proc = None
        else:
            rospy.loginfo("Neither tracker nor drivers are running")
        return EmptyResponse()



        
    def emptycb(self, data):
        rospy.logdebug("Calling empty_skel_callback()")
        if self.tracker_proc is not None:
            # then the tracker is running
            if self.first_flag:
                self.heartbeat_time = rospy.Time.now()
                self.heartbeat_samples.clear()
                self.first_flag = False
            else:
                self.heartbeat_samples.append((rospy.Time.now()-self.heartbeat_time).to_sec())
                self.heartbeat_time = rospy.Time.now()
        return


    def check_skel_process(self):
        try:
            out = self.tracker_proc.poll()
            if out is None:
                    running = True
            else:
                rospy.logwarn("check_skel_process detected that tracker process is complete")
                rospy.logwarn("This most likely indicates a segmentation fault")
                running = False
        except AttributeError:
            print "AttributeError"
        return running

    
    def checkskel_cb(self, data):
        shutdown = False
        restart = False
        # is process running properly?:
        if self.tracker_proc is not None:
            running = self.check_skel_process()
            if not running:
                self.check_count += 1
            else:
                self.check_count = 0
        if self.check_count > 5:
            rospy.logwarn("Tracker should be running and it's not")
            with open(os.path.join(os.path.expanduser("~"), "shutdown_startup.log"), "a") as fi:
                to_print = "[" + strftime("%Y-%m-%d %H:%M:%S") + "] Tracker should be running and it's not\n"
                fi.write(to_print)
            restart = True

        # how is the frequency?:
        self.skelcb_count += 1    
        if len(self.heartbeat_samples) == NUM_AVERAGES and np.mean(self.heartbeat_samples) > MAX_AVG_PERIOD:
            rospy.logwarn("Average tracker frequency has dipped below minimum")
            with open(os.path.join(os.path.expanduser("~"), "shutdown_startup.log"), "a") as fi:
                to_print = "[" + strftime("%Y-%m-%d %H:%M:%S") + "] Low frequency detected\n"
                fi.write(to_print)
            restart = True
        elif len(self.heartbeat_samples) > 0:
            if self.skelcb_count%10 == 0:
                rospy.loginfo("Average tracker frequency = %f",1/np.mean(self.heartbeat_samples))
            
        # should we attempt to restart computer?
        if (rospy.Time.now() - self.heartbeat_time).to_sec() > RESTART_TIMEOUT:
            rospy.logwarn("Detected a restart timeout")
            shutdown = True

        if shutdown or restart:
            try:
                change_resp = self.change_mode(ChangeMetaModeRequest.RESTART_KINECT)
                if not change_resp.error:
                    rospy.logerr("Tried to go back to idle mode, failed!")
                else:
                    rospy.logdebug("Back to idle mode meta mode change succeeded.")
            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s",e)
            if shutdown:
                rospy.logwarn("Shutting down computer")
                cmd = 'sudo shutdown -r now'
                subprocess.Popen(cmd,shell=True)                
            if restart:
                self.check_count = 0
                rospy.logwarn("Shutting down tracker and drivers") 
                self.stop_procs(EmptyRequest())
                rospy.logwarn("Starting tracker and drivers")
                self.start_procs(EmptyRequest())
                change_resp = self.change_mode(ChangeMetaModeRequest.IDLE_ENABLED)
                if not change_resp.error:
                    rospy.logerr("Tried to go back to idle mode, failed!")
                else:
                    rospy.logdebug("Back to idle mode meta mode change succeeded.")
        return
            
            
            
def main():
    rospy.loginfo("Starting heartbeat_tracker node...")
    rospy.init_node('heartbeat_tracker', log_level=rospy.INFO)
    try:
        hm = HeartbeatMonitor()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
    return
    


if __name__=='__main__':
    main()
    
