#!/usr/bin/env python3

import rospy
from nav2d_rospy import *

if __name__ == '__main__':
  rospy.init_node('navigator', anonymous=False)

  nav2d = Nav2D()
  rospy.spin()
