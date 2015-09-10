#!/usr/bin/env python

""" This script publishes a 10Hz visualization message that instructs rviz
    to place a sphere in front of the robot, keeping it there as the robot
    moves. """

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point, Quaternion, PoseStamped, Pose
from std_msgs.msg import Header
import rospy
import tf

rospy.init_node('marker')

listener = tf.TransformListener()

point_msg = Point(x=1.0, y=2.0, z=0.0)
quat_msg = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
pub = rospy.Publisher("/my_marker", Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    header_msg = Header(stamp=rospy.Time.now(),
                                frame_id="/base_link")

    pose_msg = Pose(position=point_msg, orientation=quat_msg)
    pose_stamped = PoseStamped(header=header_msg, pose=pose_msg)

    listener.transformPose('/base_link', pose_stamped)

    msg = Marker(header = header_msg,
                 ns = "test_ns",
                 id = 0,
                 type = Marker.SPHERE,
                 action = Marker.ADD,
                 pose = pose_msg)

    msg.scale.x = .2
    msg.scale.y = .2
    msg.scale.z = .2

    msg.color.r = 0.0
    msg.color.g = 0.0
    msg.color.b = 1.0
    msg.color.a = 1.0

    pub.publish(msg)
    r.sleep()
