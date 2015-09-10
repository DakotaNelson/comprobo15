#!/usr/bin/env python

""" This script publishes a 10Hz visualization message that instructs rviz
    to place a sphere. """

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
import rospy

rospy.init_node('marker')

point_msg = Point(x=1.0, y=2.0, z=0.0)
quat_msg = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
pose_msg = Pose(position=point_msg, orientation=quat_msg)

header_msg = Header(stamp=rospy.Time.now(),
                            frame_id="odom")
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

pub = rospy.Publisher("/my_marker", Marker, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(msg)
    r.sleep()

"""
    1 visualization_msgs::Marker marker;
    2 marker.header.frame_id = "base_link";
    3 marker.header.stamp = ros::Time();
    4 marker.ns = "my_namespace";
    5 marker.id = 0;
    6 marker.type = visualization_msgs::Marker::SPHERE;
    7 marker.action = visualization_msgs::Marker::ADD;
    8 marker.pose.position.x = 1;
    9 marker.pose.position.y = 1;
    10 marker.pose.position.z = 1;
    11 marker.pose.orientation.x = 0.0;
    12 marker.pose.orientation.y = 0.0;
    13 marker.pose.orientation.z = 0.0;
    14 marker.pose.orientation.w = 1.0;
    15 marker.scale.x = 1;
    16 marker.scale.y = 0.1;
    17 marker.scale.z = 0.1;
    18 marker.color.a = 1.0; // Don't forget to set the alpha!
    19 marker.color.r = 0.0;
    20 marker.color.g = 1.0;
    21 marker.color.b = 0.0;
    22 //only if using a MESH_RESOURCE marker type:
    23 marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    24 vis_pub.publish( marker );
"""
