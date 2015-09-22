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
        self.headingpub = rospy.Publisher('/ideal_heading', Marker, queue_size=10)
        #self.tf = TransformListener()
        self.toterr = 0
        self.kp = 1
        self.ki = .005
        self.numVectors = 0

    def callback(self, msg):
        #scan = np.array(msg.ranges[:45]+msg.ranges[361-45:])
        scan = np.array(msg.ranges)
        angles = [msg.angle_min + (i*msg.angle_increment) for i in range(len(msg.ranges))]
        angles = np.array(angles)

        # replace zeros with nans
        scan[scan == 0] = np.nan

        # turn scan data into x,y coordinate pairs
        xpoints = np.multiply(scan, np.cos(angles))
        ypoints = np.multiply(scan, np.sin(angles))

        vectors = []
        for i in range(len(xpoints)):
            if not np.isnan(xpoints[i]) and not np.isnan(ypoints[i]):
                if xpoints[i] <= 0:
                    # the obstacle is in front of us
                    # build a bunch of vectors pointing from obstacles to us (we're at 0,0)
                    # since all start at (0,0), the "-xpoints[i]" is implicitly "0-xpoints[i]"
                    vector = np.array([-xpoints[i], -ypoints[i]])
                    #length = np.linalg.norm(vector)
                    # things that are farther are lower priority
                    inverse_weight = scan[i]**2
                    vector = np.divide(vector, inverse_weight)
                    vectors.append(vector)

        self.displayVectors(vectors)

        ideal = np.array([3,0]) # we'd like to go straight ahead
        # pretty badly, too

        avoidance = np.sum(vectors,0)
        avoidance_length = np.linalg.norm(avoidance)
        avoidance = np.divide(avoidance, avoidance_length)
        '''if avoidance_length > 2:
            # avoidance_length is /2 to allow it to be double that of ideal_length
            # this means the obstacle avoidance (at most) is 2x as powerful as the goal
            avoidance = np.divide(avoidance, avoidance_length/2)'''

        goal = np.subtract(ideal, avoidance) # stay away from obstacles
        goal_length = np.linalg.norm(goal) # normalize
        goal = np.divide(goal, goal_length)
        self.displayVector(goal)

        # desired heading
        theta = np.arctan2(goal[1], goal[0])
        if np.isnan(theta): theta = 0
        # arctan(x/y) translated to ROS coords

        # simple PI controller
        cmdval = ((theta * self.kp) + (self.toterr * self.ki))
        if cmdval > 1: cmdval = 1
        print(cmdval)
        self.toterr += theta

        twister = Twist(linear=Vector3(x=0.2,y=0,z=0),angular=Vector3(x=0,y=0,z=cmdval))
        self.cmdpub.publish(twister)
        return

    def displayVector(self, vector):
        start_msg = Point(x=0, y=0, z=0.0)
        end_msg = Point(x=vector[0], y=vector[1], z=0)
        header_msg = Header(stamp=rospy.Time.now(),
                                    frame_id="base_link")
        marker = Marker(header = header_msg,
                     ns = "test_ns",
                     id = 1,
                     type = Marker.ARROW,
                     action = Marker.ADD,
                     points=[start_msg, end_msg])

        marker.scale.x = .02
        marker.scale.y = .02
        marker.scale.z = .02

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.headingpub.publish(marker)

    def displayVectors(self, vectors):
        marker_field = []
        length = max(len(vectors), self.numVectors)
        if length > self.numVectors:
            self.numVectors = length

        for i in range(length):
            if i >= len(vectors):
                vector = np.array([0,0])
                action = Marker.DELETE
            else:
                vector = vectors[i]
                action = Marker.ADD

            start_msg = Point(x=vector[0], y=vector[1], z=0.0)
            end_msg = Point(x=0, y=0, z=0)

            header_msg = Header(stamp=rospy.Time.now(),
                                        frame_id="base_link")
            marker = Marker(header = header_msg,
                         ns = "test_ns",
                         id = i,
                         type = Marker.ARROW,
                         action = action,
                         points=[start_msg, end_msg])

            marker.scale.x = .02
            marker.scale.y = .02
            marker.scale.z = .02

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_field.append(marker)

        self.goalpub.publish(marker_field)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = Avoid()
    node.run()
