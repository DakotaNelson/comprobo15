#!/usr/bin/env python

import rospy
from time import sleep
from geometry_msgs.msg import Twist, Vector3

rospy.init_node('square_dancer')

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

lin = Vector3()
rot = Vector3()

def turnLeft():
    msg = Twist(linear=lin, angular=rot)
    msg.angular.z = .8
    pub.publish(msg)
    sleep(4)
    msg.angular.z = 0.0
    pub.publish(msg)

def go1m():
    lin.x = .5
    msg = Twist(linear=lin, angular=rot)
    print(msg)
    pub.publish(msg)
    sleep(3)
    lin.x = 0.0

'''for i in range(2):
    go1m()
    turnLeft()
turnLeft()'''

go1m()

msg = Twist(linear=lin, angular=rot)
pub.publish(msg)
