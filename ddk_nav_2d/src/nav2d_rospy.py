#!/usr/bin/env python3

from numpy import rec
import rospy
import math

import tf2_ros

import actionlib

from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from nav_msgs.msg import Odometry, Path, OccupancyGrid

import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped

from kr_replanning_msgs.msg import TrackPathAction, TrackPathGoal

from kr_tracker_msgs.msg import LineTrackerAction, LineTrackerGoal, TrajectoryTrackerAction, TrajectoryTrackerGoal
from kr_tracker_msgs.srv import TransitionRequest, Transition

from ddk_nav_2d.msg import ExploreGoal, ExploreAction, ExploreResult, ExploreFeedback

from nearest_frontier_planner import *
from grid_map_rospy import *

from threading import Lock

NAV_STOP_SERVICE = "Stop"
NAV_PAUSE_SERVICE = "Pause"
NAV_GOAL_TOPIC = "goal"
NAV_STATUS_TOPIC = "nav_status"
NAV_EXPLORE_ACTION = "Explore"
NAV_EXPLORE_SERVICE = "StartExploration"

NAV_ST_IDLE = 0
NAV_ST_NAVIGATING = 1
NAV_ST_EXPLORING = 4
NAV_ST_WAITING = 5
NAV_ST_RECOVERING = 6
NAV_ST_TURNING = 7
PI = 3.14159265


class Nav2D:
  def __init__(self):
    # Param
    self.min_recheck_period_ = rospy.get_param("~min_recheck_period", 8.0)
    self.server_wait_timeout_ = rospy.get_param(
        "~server_wait_timeout", 3.0)
    self.map_frame_ = rospy.get_param("~map_frame", "ddk/odom")
    self.robot_frame_ = rospy.get_param("~robot_frame", "ddk/base_link")
    self.cell_robot_radius_ = rospy.get_param("~cell_robot_radius", 1)
    self.frequency_ = rospy.get_param("~frequency", 2.0)
    self.occupied_cell_threshold_ = rospy.get_param(
        "~occupied_cell_threshold", 0.8)
    self.explore_action_topic_ = rospy.get_param(
        "~explore_action_topic", NAV_EXPLORE_ACTION)
    self.map_inflation_radius_ = rospy.get_param(
        "~map_inflation_radius", 0.5)
    self.flight_height_ = rospy.get_param("~flight_height", 1.2)
    self.goal_recheck_ = rospy.get_param("~goal_recheck", True)
    self.obstacle_scan_range_ = rospy.get_param(
        "~obstacle_scan_range", 1.0)
    self.goal_frontier_threshold_ = rospy.get_param(
        "~goal_frontier_num_threshold", 30)
    self.frontier_distance_threshold_ = rospy.get_param(
        "~frontier_distance_threshold", 0.5)
    self.frontier_fov_ = rospy.get_param("~frontier_fov", 90.0)
    self.first_scan_ = rospy.get_param("~first_360_scan", True)
    rospy.logerr("first scan: {}".format(self.first_scan_))
    self.current_map_ = GridMap()
    self.map_updated_ = False
    self.pos_ = [0, 0, 0]
    self.roll_ = 0
    self.pitch_ = 0
    self.yaw_ = 0
    self.last_odom_t_ = 0
    self.mutex = Lock()
    self.start_point_ = None
    self.goal_point_ = None

    # Subscriber
    self.map_subscriber_ = rospy.Subscriber(
        "projected_map", OccupancyGrid, self.mapSubscriberCB, queue_size=5)
    self.pose_subscriber_ = rospy.Subscriber(
        "odom", Odometry, self.poseSubscriberCB, queue_size=10)

    # Publisher
    self.goal_publisher_ = rospy.Publisher(
        "goal_point", PoseStamped, queue_size=5)

    self.odom_publisher_ = rospy.Publisher(
        "debug_odom", PoseStamped, queue_size=5)

    # Services
    self.srv_transition_ = rospy.ServiceProxy(
        "trackers_manager/transition", Transition)
    self.jps_service_client_ = rospy.ServiceProxy(
        "jps_plan_service", GetPlan)

    # Action client
    self.line_tracker_min_jerk_client_ = actionlib.SimpleActionClient(
        "trackers_manager/line_tracker_min_jerk/LineTracker", LineTrackerAction)
    if not self.line_tracker_min_jerk_client_.wait_for_server(rospy.Duration(self.server_wait_timeout_)):
      rospy.logerr("LineTrackerMinJerk server not found.")
    self.traj_tracker_client_ = actionlib.SimpleActionClient(
        "trackers_manager/trajectory_tracker/TrajectoryTracker", TrajectoryTrackerAction)
    if not self.traj_tracker_client_.wait_for_server(rospy.Duration(self.server_wait_timeout_)):
      rospy.logerr("TrajectoryTracker server not found.")
    self.track_path_action_client_ = actionlib.SimpleActionClient(
        "TrackPathAction", TrackPathAction)
    if not self.track_path_action_client_.wait_for_server(rospy.Duration(self.server_wait_timeout_)):
      rospy.logerr("track path action server not found.")

    # tf listener
    self.tfBuffer_ = tf2_ros.Buffer()
    self.tf_listener_ = tf2_ros.TransformListener(self.tfBuffer_)

    # Init Params
    self.map_updated_ = False
    self.node_status_ = NAV_ST_IDLE
    self.line_tracker_status_ = NAV_ST_IDLE
    self.track_path_status_ = NAV_ST_IDLE
    self.traj_tracker_status_ = NAV_ST_IDLE

    # Strings for tracker transition
    self.line_tracker_min_jerk_ = "kr_trackers/LineTrackerMinJerk"
    self.traj_tracker_ = "kr_trackers/TrajectoryTracker"

    self.frontier_planner_ = FrontierPlanner(self.map_frame_)

    # Action Server
    self.explore_action_server_ = actionlib.SimpleActionServer(
        self.explore_action_topic_, ExploreAction, execute_cb=self.receiveExploreGoal, auto_start=False)
    self.explore_action_server_.start()

  def mapSubscriberCB(self, map):
    self.mutex.acquire()
    self.current_map_.update(map)
    # can add map inflation here
    self.mutex.release()
    if self.map_updated_ == False:
      rospy.loginfo("Navigator is now initialied.")
      self.current_map_.setLethalCost(self.occupied_cell_threshold_)
    self.map_updated_ = True

  def poseSubscriberCB(self, odom):
    self.pos_[0] = odom.pose.pose.position.x
    self.pos_[1] = odom.pose.pose.position.y
    self.pos_[2] = odom.pose.pose.position.z

    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
    euler = tf_conversions.transformations.euler_from_quaternion(
        quaternion)
    self.roll_ = euler[0]
    self.pitch_ = euler[1]
    self.yaw_ = euler[2]
    self.last_odom_t_ = rospy.Time.now()

  def receiveExploreGoal(self, goal):
    rospy.loginfo("Nav2D: Received explore goal.")
    if not self.map_updated_:
      rospy.logerr("No map received!")
      self.explore_action_server_.set_aborted()
      return

    if self.node_status_ != NAV_ST_IDLE:
      rospy.logwarn("Navigator is busy!")
      self.explore_action_server_.set_aborted()
      return

    self.node_status_ = NAV_ST_EXPLORING

    # Params related to recheck goal
    cycle = 0
    last_check = 0
    recheck = False
    last_frontier_time = rospy.Time.now()
    last_goal_x, last_goal_y = 0, 0

    # set to change use traj tracker / TrackPath.
    track_path = True
    moving = False
    loop_rate = rospy.Rate(self.frequency_)

    # Go Straight
    straight = 0
    while True:
      if straight >= 0.5 and self.line_tracker_status_ == NAV_ST_IDLE:
        break
      if self.line_tracker_status_ == NAV_ST_IDLE:
        self.goTo(0.5, 0, 0, 0, 0.0, 0.0, True)
        straight += 0.5
        loop_rate.sleep()

    # Turn 360, get first map
    if self.first_scan_:
      rotation = 0
      while True:
        if rotation >= 6.282 and self.line_tracker_status_ == NAV_ST_IDLE:
          break
        if self.line_tracker_status_ == NAV_ST_IDLE:
          self.goTo(0, 0, 0, 3.141, 0.0, 0.0, True)
          rotation += 3.141
        loop_rate.sleep()

    # Move to exploration target
    while True:
      if self.explore_action_server_.is_preempt_requested():
        rospy.loginfo("Exploration has been preempted externally.")
        self.explore_action_server_.set_preempted()
        return

      # Get current map Index
      if not self.getMapIndex():
        rospy.logerr(
            "Exploration failed, could not get current position.")
        self.explore_action_server_.set_aborted()
        return

      # check current status
      if self.track_path_status_ == NAV_ST_IDLE and self.traj_tracker_status_ == NAV_ST_IDLE and self.line_tracker_status_ == NAV_ST_IDLE:
        moving = False
      else:
        moving = True

      # recheck according to time
      if self.goal_recheck_:
        current_time = rospy.Time.now()
        if (current_time.to_sec() - last_frontier_time.to_sec()) > self.min_recheck_period_:
          recheck = True
        else:
          recheck = False
      else:
        recheck = False

      # Not moving(reached goal) or recheck, need to find new frontiers.
      if not moving or recheck:
        self.mutex.acquire()
        if self.preparePlan():
          result, self.goal_point_ = self.frontier_planner_.findExplorationTarget(
              self.current_map_, self.start_point_)

          if result == EXPL_TARGET_SET:
            rospy.loginfo("EXPL TARGET SET")
            last_frontier_time = rospy.Time.now()
            if recheck or not moving:
              goal_x, goal_y = self.current_map_.getCoordinates(
                  self.goal_point_)
              if goal_x != False or goal_y != False:
                map_goal_x = self.current_map_.getOriginX() + (goal_x + 0.5) * \
                    self.current_map_.getResolution()
                map_goal_y = self.current_map_.getOriginY() + (goal_y + 0.5) * \
                    self.current_map_.getResolution()
                map_goal_z = self.flight_height_

                if recheck and moving:
                  rospy.loginfo(
                      "Enter recheck exploration goal point.")
                  if (abs(last_goal_x - map_goal_x) > 0.1) or (abs(last_goal_y - map_goal_y) > 0.1):
                    self.cancelCurrentGoal()
                    rospy.loginfo("Cancel current goal.")

                # Change current status
                moving = True
                self.node_status_ = NAV_ST_EXPLORING

                last_goal_x = map_goal_x
                last_goal_y = map_goal_y

                yaw = math.atan2(
                    self.pos_[1] - map_goal_y, self.pos_[0] - map_goal_x)
                yaw += PI
                if yaw < -PI:
                  yaw += 2*PI
                if yaw > PI:
                  yaw -= 2*PI

                # Turn head, needed for trajectory tracker
                if not track_path:
                  self.line_tracker_status_ = NAV_ST_TURNING
                  while True:
                    if self.line_tracker_status_ == NAV_ST_IDLE:
                      break
                    if self.line_tracker_status_ == NAV_ST_TURNING:
                      self.goTo(self.pos_[0], self.pos_[1], self.pos_[
                                2], yaw, 0.0, 0.0, False)
                    loop_rate.sleep()

                goal_value = self.current_map_.getData(
                    self.goal_point_)

                distance = math.sqrt((map_goal_x - self.pos_[0])**2 + (
                    map_goal_y - self.pos_[1])**2 + (map_goal_z - self.pos_[2])**2)
                rospy.loginfo(
                    "Distance to goal: {}".format(distance))

                # Gen arguments needed for getJpsTraj function.
                local_time = 0.0
                # get current odom transform
                transformStamped = TransformStamped()
                try:
                  transformStamped = self.tfBuffer_.lookup_transform(
                      self.map_frame_, self.robot_frame_, rospy.Time(0))
                except:
                  rospy.loginfo(
                      "Couldn't get current odom transform")
                  return
                translation = transformStamped.transform.translation  # translation.x, y, z

                # Check what is the expected goal yaw.
                goal_yaw = self.getGoalHeading(
                    self.goal_point_)
                if goal_yaw < 0:
                  rospy.logerr("Goal heading -1.")

                tf2_quaternion = tf_conversions.transformations.quaternion_from_euler(
                    0, 0, goal_yaw)
                goal = PoseStamped()
                goal.header.frame_id = self.map_frame_
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = map_goal_x
                goal.pose.position.y = map_goal_y
                goal.pose.position.z = map_goal_z
                goal.pose.orientation.x = tf2_quaternion[0]
                goal.pose.orientation.y = tf2_quaternion[1]
                goal.pose.orientation.z = tf2_quaternion[2]
                goal.pose.orientation.w = tf2_quaternion[3]

                # Publish goal for visualization in rviz
                self.goal_publisher_.publish(goal)
                ret = self.getJpsTraj(
                    local_time, translation, goal, track_path)

          elif result == EXPL_FINISHED:
            r = ExploreResult()
            r.final_pose.x = self.pos_[0]
            r.final_pose.y = self.pos_[1]
            r.final_pose.theta = self.yaw_
            self.explore_action_server_.set_succeeded(r)
            rospy.loginfo("Exploration has finished.")

          elif result == EXPL_WAITING:
            self.node_status_ = NAV_ST_WAITING
            rospy.loginfo("Exploration is waiting.")

          elif result == EXPL_FAILED:
            rospy.loginfo("Exploration failed.")

          else:
            rospy.logerr(
                "Exploration planner returned invalid status code: {}!".format(result))

        else:
          # prepare plan failed:
          rospy.logerr("Exploration failed, prepare plan failed")
          self.explore_action_server_.set_aborted()
          return
        self.mutex.release()

      if self.node_status_ == NAV_ST_EXPLORING:
        if moving:
          if cycle % 10 == 0:
            fb = ExploreFeedback()
            fb.robot_pose.x = self.pos_[0]
            fb.robot_pose.y = self.pos_[1]
            fb.robot_pose.theta = self.yaw_
            self.explore_action_server_.publish_feedback(fb)

      # sleep reaming time:
      cycle += 1
      loop_rate.sleep()

  def goTo(self, x, y, z, yaw, v_des, a_des, relative):
    goal = LineTrackerGoal()
    goal.x = x
    goal.y = y
    goal.relative = relative
    # Convert relative translation in body frame to global frame
    if relative:
      goal.x = x * math.cos(self.yaw_) - y * math.sin(self.yaw_)
      goal.y = x * math.sin(self.yaw_) + y * math.cos(self.yaw_)
    goal.z = z
    goal.yaw = yaw
    goal.v_des = v_des
    goal.a_des = a_des

    self.line_tracker_min_jerk_client_.send_goal(
        goal, done_cb=self.lineTrackerDoneCB)
    self.line_tracker_status_ = NAV_ST_NAVIGATING
    return self.transition(self.line_tracker_min_jerk_)

  def trackPath(self, planned_path, method):
    if method:
      path_goal = TrackPathGoal()
      path_goal.path = planned_path
      self.track_path_action_client_.send_goal(
          path_goal, done_cb=self.trackPathDoneCB)
      self.track_path_status_ = NAV_ST_NAVIGATING
      rospy.loginfo("Send Track Path goal to server.")
      return True
    else:
      path_goal = TrajectoryTrackerGoal()
      for i in range(0, planned_path.poses.size()):
        path_goal.waypoints.push_back(planned_path.poses[i].pose)
      self.traj_tracker_client_.send_goal(
          path_goal, done_cb=self.trajTrackerDoneCB)
      self.traj_tracker_status_ = NAV_ST_NAVIGATING
      rospy.loginfo("Send trajectory tracker goal to server.")
      return self.transition(self.traj_tracker_)
    return True

  def cancelCurrentGoal(self):
    if self.track_path_status_ != NAV_ST_IDLE:
      self.track_path_action_client_.cancel_goal()
      self.track_path_status_ = NAV_ST_IDLE
    if self.traj_tracker_status_ != NAV_ST_IDLE:
      self.traj_tracker_client_ptr_.cancel_goal()
      self.traj_tracker_status_ = NAV_ST_IDLE
    if self.line_tracker_status_ != NAV_ST_IDLE:
      self.line_tracker_min_jerk_client_ptr_.cancel_goal()
      self.line_tracker_status_ = NAV_ST_IDLE
    rospy.logwarn("Cancel current goal")

  def lineTrackerDoneCB(self, state, result):
    rospy.loginfo("Goal reached.")
    self.line_tracker_status_ = NAV_ST_IDLE

  def trackPathDoneCB(self, state, result):
    rospy.loginfo("trackPathDoneCB.")
    self.track_path_status_ = NAV_ST_IDLE

  def trajTrackerDoneCB(self, state, result):
    rospy.loginfo("trajTracker goal reached.")
    self.traj_tracker_status_ = NAV_ST_IDLE

  def transition(self, tracker_str):
    transition_cmd = TransitionRequest()
    transition_cmd.tracker = tracker_str
    resp = self.srv_transition_(transition_cmd)
    if resp:
      rospy.loginfo("Current tracker: {}".format(tracker_str))
      return True
    return False

  def getMapIndex(self):
    transform_stamped = TransformStamped()
    try:
      transform_stamped = self.tfBuffer_.lookup_transform(
          self.map_frame_, self.robot_frame_, rospy.Time(0))
    except:
      rospy.logwarn("Couldn't get robot position")
      return False

    world_x = transform_stamped.transform.translation.x
    world_y = transform_stamped.transform.translation.y

    current_x = (world_x - self.current_map_.getOriginX()
                 ) // self.current_map_.getResolution()
    current_y = (world_y - self.current_map_.getOriginY()
                 ) // self.current_map_.getResolution()
    start_point = self.current_map_.getIndex(current_x, current_y)
    self.start_point_ = start_point

    return True

  def preparePlan(self):
    if not self.getMapIndex():
      return False

    # clear robot footprint in map
    x, y = self.current_map_.getCoordinates(self.start_point_)
    if x != False:
      for i in range(-self.cell_robot_radius_, self.cell_robot_radius_):
        for j in range(-self.cell_robot_radius_, self.cell_robot_radius_):
          self.current_map_.setDataByCoord(x+i, y+j, 0)

    rospy.loginfo(
        "prepare plan ready, set Startpoint as {}".format(self.start_point_))

    # debug
    current_x, current_y = self.current_map_.getCoordinates(
        self.start_point_)
    current_world_x = self.current_map_.getOriginX() + (current_x + 0.5) * \
        self.current_map_.getResolution()
    current_world_y = self.current_map_.getOriginY() + (current_y + 0.5) * \
        self.current_map_.getResolution()
    my_odom = PoseStamped()
    my_odom.header.frame_id = self.map_frame_
    my_odom.header.stamp = rospy.Time.now()
    my_odom.pose.position.x = current_world_x
    my_odom.pose.position.y = current_world_y
    my_odom.pose.position.z = 0
    my_odom.pose.orientation.x = 0
    my_odom.pose.orientation.y = 0
    my_odom.pose.orientation.z = 0
    my_odom.pose.orientation.w = 1
    self.odom_publisher_.publish(my_odom)

    return True

  def getJpsTraj(self, traj_time, translation, min_cost_pt, method):
    jps_srv = GetPlanRequest()
    jps_srv.start.header = min_cost_pt.header
    jps_srv.start.pose.position.x = translation.x
    jps_srv.start.pose.position.y = translation.y
    jps_srv.start.pose.position.z = translation.z
    jps_srv.start.pose.orientation.x = min_cost_pt.pose.orientation.x
    jps_srv.start.pose.orientation.y = min_cost_pt.pose.orientation.y
    jps_srv.start.pose.orientation.z = min_cost_pt.pose.orientation.z
    jps_srv.start.pose.orientation.w = min_cost_pt.pose.orientation.w

    jps_srv.goal = min_cost_pt

    resp = self.jps_service_client_(jps_srv)
    if resp:
      rospy.loginfo("Successful jps service call")
      if len(resp.plan.poses) == 0:
        rospy.logerr("JPS did not return a path")
        return False
      self.trackPath(resp.plan, method)
      return True
    else:
      rospy.logerr("Failed to jps call service")
      return False

  def getGoalHeading(self, goal_index):
    map_width = self.current_map_.getWidth()
    y = int(goal_index // map_width)
    x = int(goal_index % map_width)
    if self.current_map_.getDataByCoord(x-1, y-1) == -1:
      return 225*PI/180
    if self.current_map_.getDataByCoord(x-1, y) == -1:
      return 180*PI/180
    if self.current_map_.getDataByCoord(x-1, y+1) == -1:
      return 135*PI/180
    if self.current_map_.getDataByCoord(x, y-1) == -1:
      return 270*PI/180
    if self.current_map_.getDataByCoord(x, y+1) == -1:
      return 90*PI/180
    if self.current_map_.getDataByCoord(x+1, y-1) == -1:
      return 315*PI/180
    if self.current_map_.getDataByCoord(x+1, y) == -1:
      return 0.0
    if self.current_map_.getDataByCoord(x+1, y+1) == -1:
      return 45*PI/180
    return -1.0
