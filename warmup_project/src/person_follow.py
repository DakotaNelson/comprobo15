#!/usr/bin/env python
""" Causes the Neato to follow the center of mass of the points in front of it """

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Vector3, PointStamped, Point, Quaternion, Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf import TransformListener

class PersonFollow(object):
    def __init__(self):
        rospy.init_node('person_follow')
        rospy.Subscriber('scan', LaserScan, self.callback, queue_size=1)
        self.cmdpub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goalpub = rospy.Publisher('/mass_center', Marker, queue_size=10)
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

        xpoints = np.multiply(scan, np.cos(angles))
        ypoints = np.multiply(scan, np.sin(angles))

        xbox = []
        ybox = []
        for i in range(len(xpoints)):
            #print("({},{})".format(xpoints[i], ypoints[i]))
            if xpoints[i] <= 5 and xpoints[i] >= 0.05:
                if ypoints[i] >= -1.5 and ypoints[i] <= 1.5:
                    xbox.append(xpoints[i])
                    ybox.append(ypoints[i])

        xmass = np.nanmean(xbox)
        ymass = np.nanmean(ybox)

        #print("center of mass: ({}, {})".format(xmass, ymass))

        point_msg = Point(x=xmass, y=ymass, z=0.0)
        quat_msg = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        pose_msg = Pose(position=point_msg, orientation=quat_msg)
        header_msg = Header(stamp=rospy.Time.now(),
                                    frame_id="base_link")
        marker = Marker(header = header_msg,
                     ns = "test_ns",
                     id = 0,
                     type = Marker.SPHERE,
                     action = Marker.ADD,
                     pose = pose_msg)

        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.goalpub.publish(marker)

        if np.isnan(xmass):
            xmass = 0
        if np.isnan(ymass):
            ymass = 0
        xerr = xmass
        yerr = ymass
        self.totxerr += xerr
        self.totyerr += yerr

        # simple PI controller
        xcmdval = ((xerr * self.kpx) + (self.totxerr * self.kix))
        ycmdval = ((yerr * self.kpy) + (self.totyerr * self.kiy))

        #print(xcmdval)
        #print(ycmdval)
        twister = Twist(linear=Vector3(x=xcmdval,y=0,z=0),angular=Vector3(x=0,y=0,z=ycmdval))
        self.cmdpub.publish(twister)
        return

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = PersonFollow()
    node.run()
