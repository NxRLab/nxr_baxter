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

class Heartbeat_Monitor:
    def __init__(self):
        # For calculating skeleton heartbeat
        # Average it every 5 seconds.
        self.heartbeat_period = 5.0
        # Initialize count
        self._heartbeat_count = 0

        self.n_moving_avg_filt = 12
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

    #Callback for the heartbeat. Updates the heartbeat count.
    #There will be a separate timer to calculate the actual frequency
    def heartbeatCallback(self, event):
        self._heartbeat_count += 1

    # Callback for heartbeat timer calculation. Gets the current count and will
    # calculate the average. For now it will just asdflkjsd
    def heartbeat_timer_callback(self, event):
        # Calculate frequency
        ht_bt_freq = self._heartbeat_count/self.heartbeat_period
        self.freq_filter_list.push(ht_bt_freq)
        #Reset count
        self._heartbeat_count = 0
        print self.freq_filter_list.sum/self.n_moving_avg_filt


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
    
    rospy.spin()

    print("\nHeartbeat Tracker shutting down.")
