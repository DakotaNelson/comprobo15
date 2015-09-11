#!/usr/bin/env python

import tty
import select
import sys
import termios

import rospy
from geometry_msgs.msg import Twist, Vector3

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

rospy.init_node('custom_teleop')

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

settings = termios.tcgetattr(sys.stdin)
key = None
while key != '\x03':
#r = rospy.Rate(10)
#while not rospy.is_shutdown():
    key = getKey()
    '''
    w 119
    a 97
    s 115
    d 100
    '''

    lin = Vector3()
    rot = Vector3()

    okey = ord(key)
    if okey == 119:
        #forward
        lin.x = .5
    elif okey == 97:
        # left
        rot.z = .8
    elif okey == 115:
        # back
        lin.x = -.5
    elif okey == 100:
        # right
        rot.z = -.8

    msg = Twist(linear=lin, angular=rot)
    print(msg)
    pub.publish(msg)
    #r.sleep()
