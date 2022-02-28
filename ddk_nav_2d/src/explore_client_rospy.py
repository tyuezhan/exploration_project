#!/usr/bin/env python3

import rospy
import actionlib
from ddk_nav_2d.msg import ExploreGoal, ExploreAction
from std_srvs.srv import Trigger, TriggerResponse

client = actionlib.SimpleActionClient("Explore", ExploreAction)


def receiveCommand(req):
  goal = ExploreGoal()
  client.send_goal(goal)
  res = TriggerResponse()
  res.success = True
  res.message = "Send ExploreGoal to Navigator."
  return res


if __name__ == "__main__":
  rospy.init_node('Explore', anonymous=False)
  cmdServer = rospy.Service("StartExploration", Trigger, receiveCommand)

  client.wait_for_server()
  rospy.spin()
