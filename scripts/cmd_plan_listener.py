#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from nav_msgs.msg import Path
from nav2d_operator.msg import cmd
from waypoint_publisher import waypoint_publisher

class cmd_plan_listenser():
    def __init__(self):
        
        rospy.Subscriber('Navigator/plan', GridCells, self.gridcellscallback)
        rospy.Subscriber('cmd', cmd, self.cmdcallback)
        self.publisher = waypoint_publisher()

    def gridcellscallback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + 'gridcell heard %s', data)
        print(data.cells)

    def cmdcallback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + 'cmd heard %s', data)
        print(data.Turn)

if __name__ == '__main__':
    rospy.init_node('cmd_to_path', anonymous=True)

    mySub = cmd_plan_listenser()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
