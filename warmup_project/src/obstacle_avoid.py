#!/usr/bin/env python
""" Causes the Neato to follow the center of mass of the points in front of it """

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3, PointStamped, Point, Quaternion, Pose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf import TransformListener

class Avoid(object):
    def __init__(self):
        rospy.init_node('person_follow')
        rospy.Subscriber('scan', LaserScan, self.callback, queue_size=1)
        self.cmdpub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goalpub = rospy.Publisher('/vector_field', MarkerArray, queue_size=10)
        self.tf = TransformListener()
        self.totxerr = 0
        self.totyerr = 0
        self.kpx = .2
        self.kpy = 1
        self.kix = .005
        self.kiy = .005

    def callback(self, msg):
        #scan = np.array(msg.ranges[:45]+msg.ranges[361-45:])
        scan = np.array(msg.ranges)
        angles = [msg.angle_min + (i*msg.angle_increment) for i in range(len(msg.ranges))]
        angles = np.array(angles)
        #angles = np.array(angles[:45]+angles[361-45:])

        # replace zeros with nans
        scan[scan == 0] = np.nan

        # turn scan data into x,y coordinate pairs
        xpoints = np.multiply(scan, np.cos(angles))
        ypoints = np.multiply(scan, np.sin(angles))

        vectors = []
        for i in range(len(xpoints)):
            # build a bunch of vectors pointing from obstacles to us (we're at 0,0)
            vector = np.array([-xpoints[i], -ypoints[i]])
            #length = np.linalg.norm(vector)
            # things that are farther are lower priority
            #vector = np.divide(scan[i])
            if not np.isnan(xpoints[i]) and not np.isnan(ypoints[i]):
                vectors.append(vector)

        self.displayVectors(vectors)

        """xerr = xmass
        yerr = ymass
        self.totxerr += xerr
        self.totyerr += yerr

        # simple PI controller
        xcmdval = ((xerr * self.kpx) + (self.totxerr * self.kix))
        ycmdval = ((yerr * self.kpy) + (self.totyerr * self.kiy))

        twister = Twist(linear=Vector3(x=xcmdval,y=0,z=0),angular=Vector3(x=0,y=0,z=ycmdval))
        #self.cmdpub.publish(twister)"""
        return

    def displayVectors(self, vectors):
        marker_field = []
        counter = 0
        for vector in vectors:
            start_msg = Point(x=vector[0], y=vector[1], z=0.0)
            end_msg = Point(x=0, y=0, z=0)
            header_msg = Header(stamp=rospy.Time.now(),
                                        frame_id="base_link")
            marker = Marker(header = header_msg,
                         ns = "test_ns",
                         id = counter,
                         type = Marker.ARROW,
                         action = Marker.ADD,
                         points=[start_msg, end_msg])

            marker.scale.x = .02
            marker.scale.y = .02
            marker.scale.z = .02

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            counter += 1

            marker_field.append(marker)

        self.goalpub.publish(marker_field)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = Avoid()
    node.run()
