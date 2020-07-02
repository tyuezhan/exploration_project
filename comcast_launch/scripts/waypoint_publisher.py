#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class waypoint_publisher():
    def __init__(self):
        
        self.pub = rospy.Publisher('ddk/waypoints', Path, queue_size=10)

    def publish_waypoint(self, header, ):
        # rospy.loginfo(xxxx)
        
        pose = Pose()
        point = Point()
        quaternion = Quaternion()
        pose.position = point
        pose.orientation = quaternion
        msg_poseStamped = PoseStamped()
        msg_poseStamped.pose = pose
        msg_header = header

        msg = Path()
        msg.header = msg_header
        msg.poses = msg_poseStamped
        self.pub.publish("111")

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher', anonymous=True)
    waypoint_pub = waypoint_publisher()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
