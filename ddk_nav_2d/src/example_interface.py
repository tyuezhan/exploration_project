#! /usr/bin/env python3
import rospy
import numpy as np
import random
import tf2_ros
import tf_conversions
import actionlib

from threading import Lock

from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from kr_tracker_msgs.msg import TrajectoryTrackerAction, TrajectoryTrackerGoal, CircleTrackerAction, CircleTrackerGoal
from kr_replanning_msgs.msg import TrackPathAction, TrackPathGoal

from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from std_srvs.srv import SetBool, Trigger

from kr_python_interface.mav_interface import KrMavInterface
from nearest_frontier_planner import *
from grid_map_rospy import *

class KrInterface:
  def __init__(self):

    # Params
    self.map_frame_ = rospy.get_param("~map_frame", "ddk/odom")
    self.robot_frame_ = rospy.get_param("~robot_frame", "ddk/base_link")
    self.cell_robot_radius_ = rospy.get_param("~cell_robot_radius", 1)
    self.server_wait_timeout_ = rospy.get_param("~server_wait_timeout", 3.0)
    self.frequency_ = rospy.get_param("~frequency", 2.0)
    self.occupied_cell_threshold_ = rospy.get_param("~occupied_cell_threshold", 0.8)
    self.map_inflation_radius_ = rospy.get_param("~map_inflation_radius", 0.5)
    self.flight_height_ = rospy.get_param("~flight_height", 1.2)
    self.goal_recheck_ = rospy.get_param("~goal_recheck", True)
    self.obstacle_scan_range_ = rospy.get_param("~obstacle_scan_range", 1.0)
    self.goal_frontier_threshold_ = rospy.get_param("~goal_frontier_num_threshold", 30)
    self.frontier_distance_threshold_ = rospy.get_param("~frontier_distance_threshold", 0.5)
    self.frontier_fov_ = rospy.get_param("~frontier_fov", 90.0)

    # Init Params
    self.map_updated_ = False
    self.mutex_ = Lock()
    self.current_map_ = GridMap()
    self.frontier_planner_ = FrontierPlanner(self.map_frame_)

    # Creating MAV objects using Kr Python interface
    self.mav_namespace = 'ddk'
    self.mav_id = 1
    self.mav_obj = KrMavInterface(self.mav_namespace, self.mav_id)

    # tf listener
    self.tfBuffer_ = tf2_ros.Buffer()
    self.tf_listener_ = tf2_ros.TransformListener(self.tfBuffer_)

    # Subscribers
    self.map_subscriber_ = rospy.Subscriber("projected_map", OccupancyGrid, self.map_subscriber_cb, queue_size=5)

    # Publishers
    self.goal_publisher_ = rospy.Publisher("goal_point", PoseStamped, queue_size=5)
    self.odom_publisher_ = rospy.Publisher("debug_odom", PoseStamped, queue_size=5)

    # Services
    self.jps_service_client_ = rospy.ServiceProxy("jps_plan_service", GetPlan)

    # Action clients
    self.track_path_action_client_ = actionlib.SimpleActionClient("TrackPathAction", TrackPathAction)
    if not self.track_path_action_client_.wait_for_server(rospy.Duration(self.server_wait_timeout_)):
      rospy.logerr("track path action server not found.")

    #Timer
    self.tm = rospy.Timer(rospy.Duration(1.0/self.frequency_), self.do_work)

  def map_subscriber_cb(self, map):
    self.mutex_.acquire()
    self.current_map_.update(map)
    # can add map inflation here
    self.mutex_.release()
    if self.map_updated_ == False:
      self.current_map_.setLethalCost(self.occupied_cell_threshold_)
    self.map_updated_ = True

  def do_work(self, event):

    rospy.logwarn('Timer called at ' + str(event.current_real))

    #Get the current map (subscribed)

    # Get current location map Index
    if not self.get_map_index():
      rospy.logerr("Exploration failed, could not get current position.")
      return

    #TODO inflate the map
    #TODO Scale the map for inference

    #Get the frontiers/next waypoint (frontier or NN)
    ret, waypoint = self.get_frontier_waypoint()
    if not ret:
      rospy.logerr('Did not get a valid waypoint')
      return

    #Sanitize/check the generated waypoint

    #Generate a 2D path to the waypoint

    current_pose = TransformStamped()
    try:
      current_pose = self.tfBuffer_.lookup_transform(self.map_frame_, self.robot_frame_, rospy.Time(0))
    except:
      rospy.logwarn("Couldn't get robot position")
      return

    jps_srv = GetPlanRequest()
    jps_srv.start.header = current_pose.header
    jps_srv.start.pose.position.x = current_pose.transform.translation.x
    jps_srv.start.pose.position.y = current_pose.transform.translation.y
    jps_srv.start.pose.position.z = current_pose.transform.translation.z
    jps_srv.start.pose.orientation.x = current_pose.transform.rotation.x
    jps_srv.start.pose.orientation.y = current_pose.transform.rotation.y
    jps_srv.start.pose.orientation.z = current_pose.transform.rotation.z
    jps_srv.start.pose.orientation.w = current_pose.transform.rotation.w

    goal = PoseStamped()
    goal.header.frame_id = self.map_frame_
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = waypoint[0]
    goal.pose.position.y = waypoint[1]
    goal.pose.position.z = self.flight_height_
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 1

    jps_srv.goal = goal

    jps2d_resp = self.jps_service_client_(jps_srv)
    if jps2d_resp:
      rospy.loginfo("Successful jps service call")
      if len(jps2d_resp.plan.poses) == 0:
        rospy.logerr("JPS did not return a path")
        return
    else:
      rospy.logerr("Failed to jps call service")
      return

    #Track the generated 2D path
    path_goal = TrackPathGoal()
    path_goal.path = jps2d_resp.plan
    self.track_path_action_client_.send_goal(path_goal, done_cb=self.trackpath_done_cb)

    #Rinse and repeat every X seconds

  def get_frontier_waypoint(self):
    self.mutex_.acquire()
    result, self.goal_point_ = self.frontier_planner_.findExplorationTarget(self.current_map_, self.start_point_)
    self.mutex_.release()

    if self.prepare_plan():
      if result == EXPL_TARGET_SET:
        rospy.loginfo("Exploration target set")

        goal_x, goal_y = self.current_map_.getCoordinates(self.goal_point_)
        if goal_x != False or goal_y != False:
          map_goal_x = self.current_map_.getOriginX() + (goal_x + 0.5) * self.current_map_.getResolution()
          map_goal_y = self.current_map_.getOriginY() + (goal_y + 0.5) * self.current_map_.getResolution()

        return True, [map_goal_x, map_goal_y]

      elif result == EXPL_FINISHED:
        rospy.loginfo("Exploration has finished.")
        return False, []

      elif result == EXPL_WAITING:
        rospy.loginfo("Exploration is waiting.")
        return False, []

      elif result == EXPL_FAILED:
        rospy.loginfo("Exploration failed.")
        return False, []
      else:
        rospy.logerr("Exploration planner returned invalid status code: {}!".format(result))
        return False, []
    else:
      rospy.logerr("Exploration failed, prepare plan failed")
      return False, []

  def trackpath_done_cb(self, state, result):
    rospy.loginfo("trackPath done")

  def prepare_plan(self):
    if not self.get_map_index():
      return False

    # clear robot footprint in map
    x, y = self.current_map_.getCoordinates(self.start_point_)
    if x != False:
      for i in range(-self.cell_robot_radius_, self.cell_robot_radius_):
        for j in range(-self.cell_robot_radius_, self.cell_robot_radius_):
          self.current_map_.setDataByCoord(x+i, y+j, 0)

    rospy.loginfo(
        "prepare plan ready, set Startpoint as {}".format(self.start_point_))
    return True

  def get_map_index(self):
    transform_stamped = TransformStamped()
    try:
      transform_stamped = self.tfBuffer_.lookup_transform(self.map_frame_, self.robot_frame_, rospy.Time(0))
    except:
      rospy.logwarn("Couldn't get robot position")
      return False

    world_x = transform_stamped.transform.translation.x
    world_y = transform_stamped.transform.translation.y

    current_x = (world_x - self.current_map_.getOriginX()) // self.current_map_.getResolution()
    current_y = (world_y - self.current_map_.getOriginY()) // self.current_map_.getResolution()
    start_point = self.current_map_.getIndex(current_x, current_y)
    self.start_point_ = start_point

    return True

if __name__ == '__main__':
  rospy.init_node('example_interface', anonymous=False)

  kr_interface = KrInterface()
  #kr_interface.do_work()
  rospy.spin()

