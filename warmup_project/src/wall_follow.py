#!/usr/bin/env python
""" Causes the Neato to follow a nearby wall...
    as long as the wall is to its right and there isn't much else around. """

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class WallFollow(object):
    def __init__(self):
        rospy.init_node('wall_follow')
        rospy.Subscriber('scan', LaserScan, self.callback, queue_size=1)
        self.cmdpub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scanpub = rospy.Publisher('/average_scan', LaserScan, queue_size=10)
        self.wallDist = None
        self.toterr = 0 # cumulative error (I in PID)
        self.kp = .6
        self.ki = .005

    def callback(self, msg):
        offset = 270 # skip the first n degrees of data
        scan = np.array(msg.ranges[offset:])
        angles = [msg.angle_min + np.radians(offset) + (i*msg.angle_increment) for i in range(len(scan))]
        # create a moving average of the scan with a window of 5
        """avg = self.movingAvg(scan, 5)
        newMsg = msg
        newMsg.ranges = avg
        self.scanpub.publish(newMsg)"""
        # replace zeros with nans
        scan[scan == 0] = np.nan
        # find the smallest scan distance
        minindex = np.nanargmin(scan)
        minval = scan[minindex]
        minangle = angles[minindex]
        print("min: {} at angle {}".format(minval, np.degrees(minangle)))

        # find the desired hypotenuse length
        # cos(theta) = x/h
        # where x = distance to wall and h = scan length at angle theta
        #desired = minval / np.cos(np.subtract(angles,minangle))
        #angles -= np.radians(90)
        print("angles:")
        print(angles)
        print("desired:")
        desired = np.arccos(minval/angles)
        print(desired)
        # actual angle according to scan
        actual = np.arccos(minval/scan)
        print(actual)
        err = np.nanmean(desired - actual)
        if err is np.nan:
            err = 0
        self.toterr += err

        # simple PI controller
        cmdval = ((err * self.kp) + (self.toterr * self.ki))

        print(cmdval)
        # negative = turn right
        # positive = turn left
        twister = Twist(linear=Vector3(x=0.0,y=0,z=0),angular=Vector3(x=0,y=0,z=cmdval))
        #self.cmdpub.publish(twister)
        return

    def movingAvg(self, data, window_size):
        window = np.ones(int(window_size))/float(window_size)
        return np.convolve(data, window, 'same')

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = WallFollow()
    node.run()
