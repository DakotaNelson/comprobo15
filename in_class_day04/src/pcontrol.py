#!/usr/bin/env python
""" Attempts to move the Neato exactly 1 m forward using wheel odometry """

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class MoveDistance(object):
    def __init__(self, distance):
        rospy.init_node('move_distance')
        rospy.Subscriber('odom', Odometry, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.distance = distance
        self.startx = None

    def callback(self, msg):
        xpos = msg.pose.pose.position.x
        if self.startx is None:
            self.startx = xpos
            print("Started at {}".format(self.startx))
        if xpos <= self.startx + self.distance:
            # keep going
            twister = Twist(linear=Vector3(x=0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
            self.pub.publish(twister)
            return True
        # stop
        twister = Twist(linear=Vector3(x=0.0,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
        self.pub.publish(twister)
        return False

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = MoveDistance(1)
    node.run()
