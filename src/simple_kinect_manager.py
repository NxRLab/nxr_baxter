#!/usr/bin/env python

import rospy
import roslib
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
import os
import signal
import subprocess
import threading


def check_if_kinect_running():
    


def process_get_children(node):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % int(node), shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    return ps_output.split("\n")[:-1]
    
def process_get_children_recursively(node, leafs):
    if node is not None:
        children = process_get_children(node)
        if len(children) == 0:
            leafs.append(node)
        for n in children:
            process_get_children_recursively(n, leafs)

def terminate_process_and_children(p):
    rospy.loginfo("Calling terminate_process_and_children")
    plist = []
    try:
        pnum = p.pid
    except AttributeError:
        pnum = p
    process_get_children_recursively(pnum, plist)
    for pid_str in plist:
        os.kill(int(pid_str), signal.SIGINT)
    try:
        p.kill()
    except AttributeError:
        os.kill(int(p), signal.SIGINT)
    return


class KinectController( object ):
    def __init__(self):
        rospy.logdebug("Starting KinectController()")
        # create empty processes:
        self.openni_proc = None
        self.skel_tracker_proc = None

        # create services for starting and stopping Kinect:
        self.start_kin_service = rospy.Service('start_kinect', Empty, self.start_procs)
        self.stop_kin_service = rospy.Service('stop_kinect', Empty, self.stop_procs)
        return

    def start_procs(self, req):
        rospy.loginfo("Start service called")
        if self.openni_proc == None or self.openni_proc.poll() != None:
            rospy.loginfo("Launching openni processes...")
            cmd = 'roslaunch openni_launch openni.launch'
            self.openni_proc = subprocess.Popen(cmd, shell=True)
            rospy.sleep(4.0)
            rospy.loginfo("PROCESS IS = %s"%self.openni_proc.pid)
        else:
            rospy.logwarn("Trying to start openni thread while it is already running.")
            rospy.loginfo("self.openni_proc.poll() returns %s " % self.openni_proc.poll())
        return EmptyResponse()

    def stop_procs(self, req):
        rospy.loginfo("Stop service called")
        if self.openni_proc != None:
            rospy.loginfo("process is not None... attempt kill")
            # p2 = subprocess.Popen("rosnode kill /camera/camera_nodelet_manager", shell=True, stdout=subprocess.PIPE)
            # rospy.sleep(2.0)

            # Kill openni_launch
            rospy.loginfo("Killing openni processes...")
            terminate_process_and_children(self.openni_proc)

            # # USB restart
            # pkgdir = roslib.packages.get_pkg_dir('nxr_baxter')
            # rospy.loginfo("Restarting usb...")
            # cmd = os.path.join(pkgdir, "src/restart_usb.sh")
            # subprocess.call(cmd, shell=True)

            # # set the flag in udev_reload.txt for the cron job to fix it
            # file_name = os.path.join(os.path.expanduser("~"),'src/udev_reload.txt')
            # checked = False
            # while not checked:
            #     try:
            #         fo = open(file_name, 'w+')
            #         fo.write("1")
            #         fo.close()
            #         checked = True
            #     except IOError:
            #         rospy.logwarn("Could not open %s"%file_name)
            #         pass

            self.openni_proc = None
        else:
            rospy.loginfo("openni_proc is None")
        return EmptyResponse()


def main():
    rospy.init_node('kinect_control', log_level=rospy.INFO)
    rospy.loginfo('kinect_control node running')
    try:
        controller = KinectController()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
