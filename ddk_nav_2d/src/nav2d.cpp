#include <ddk_nav_2d/nav2d.h>
#include <ros/console.h>
#include <string>

#include <math.h>

using namespace ros;

#define PI 3.14159265

Nav2D::Nav2D() {

  NodeHandle nh;

  // Subscriber
  map_subscriber_ = nh.subscribe("projected_map", 5, &Nav2D::mapSubscriberCB, this);
  pose_subscriber_ = nh.subscribe("ddk/ground_truth/odom", 5000, &Nav2D::poseSubscriberCB, this);

  // Load planer
  nh.param("min_replanning_period", min_replanning_period_, 5.0);
  nh.param("max_replanning_period", max_replanning_period_, 1.0);
  nh.param("exploration_strategy", exploration_strategy_, std::string("NearestFrontierPlanner"));
  try {
    plan_loader_ = new PlanLoader("nav2d_navigator", "ExplorationPlanner");
    exploration_planner_ = plan_loader_->createInstance(exploration_strategy_);
    ROS_INFO("Successfully loaded exploration strategy [%s].", exploration_strategy_.c_str());

  } catch (pluginlib::PluginlibException &ex) {
    ROS_ERROR("Failed to load exploration plugin! Error: %s", ex.what());
    explore_action_server_ = NULL;
    plan_loader_ = NULL;
  }

  // Explore action server
  nh.param("explore_action_topic", explore_action_topic_, std::string(NAV_EXPLORE_ACTION));
  explore_action_server_ = new ExploreActionServer(explore_action_topic_, boost::bind(&Nav2D::receiveExploreGoal, this, _1), false);
  explore_action_server_->start();

  // Get first map action server
  nh.param("getmap_action_topic", get_map_action_topic_, std::string(NAV_GETMAP_ACTION));
  get_first_map_action_server_ = new GetFirstMapActionServer(get_map_action_topic_, boost::bind(&Nav2D::receiveGetMapGoal, this, _1), false);
  get_first_map_action_server_->start();

  node_status_ = NAV_ST_IDLE;
  line_tracker_Status_ = NAV_ST_IDLE;
  line_tracker_min_jerk_client_ = new ClientType(nh, "ddk/trackers_manager/line_tracker_min_jerk/LineTracker", true);
  float server_wait_timeout;
  nh.param("Server_wait_timeout", server_wait_timeout, 0.5f);
  if (!line_tracker_min_jerk_client_->waitForServer(ros::Duration(server_wait_timeout))) {
    ROS_ERROR("LineTrackerMinJerk server not found.");
  }

  // Services
  line_tracker_min_jerk_ = "kr_trackers/LineTrackerMinJerk";
  active_tracker_ = "";
  srv_transition_ = nh.serviceClient<kr_tracker_msgs::Transition>("ddk/trackers_manager/transition");

  // tf listener
  nh.param("map_frame", map_frame_, std::string("ddk/odom"));
  nh.param("robot_frame", robot_frame_, std::string("ddk/base_link"));
  robot_frame_ = tf_Listener_.resolve(robot_frame_);
  map_frame_ = tf_Listener_.resolve(map_frame_);

  // Planning related param
  nh.param("map_inflation_radius", inflation_radius_, 1.0);
  nh.param("robot_radius", robot_radius_, 0.2);
  cost_obstacle_ = 100;
  // cost_lethal_ = (3.0 - (robot_radius_ / inflation_radius_)) * (double)cost_obstacle_;
  cost_lethal_ = 100;
  map_updated_ = false;

  // replanning
  jps_service_client_ = nh.serviceClient<nav_msgs::GetPlan>("ddk/jps_plan_service");

  track_path_status_ = NAV_ST_IDLE;
  track_path_action_client_ = new trackPathClientType(nh, "ddk/TrackPathAction", true);
  if (!track_path_action_client_->waitForServer(ros::Duration(server_wait_timeout))) {
    ROS_ERROR("track path action server not found.");
  }
}

Nav2D::~Nav2D() {
  ROS_INFO("Clean.");
  delete explore_action_server_;
  delete get_first_map_action_server_;
  exploration_planner_.reset();
  delete plan_loader_;
  delete line_tracker_min_jerk_client_;
  delete track_path_action_client_;
}

void Nav2D::mapSubscriberCB(const nav_msgs::OccupancyGrid &map) {
  ROS_INFO("Map update received.");
  current_map_.update(map);

  // if(mCellInflationRadius == 0){
  // ROS_INFO("Navigator is now initialized.");
  // mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();
  // mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();
  // 	mInflationTool.computeCaches(mCellInflationRadius);
  // mCurrentMap.setLethalCost(mCostLethal);
  // }

  if (map_updated_ == false) {
    ROS_INFO("Navigator is now initialized.");
    cell_robot_radius_ = 10;
    current_map_.setLethalCost(cost_lethal_);
  }
  map_updated_ = true;
}

void Nav2D::poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom) {
  // ROS_INFO("pose update received.");
  pos_(0) = odom->pose.pose.position.x;
  pos_(1) = odom->pose.pose.position.y;
  pos_(2) = odom->pose.pose.position.z;

  vel_(0) = odom->twist.twist.linear.x;
  vel_(1) = odom->twist.twist.linear.y;
  vel_(2) = odom->twist.twist.linear.z;

  odom_q_ = Quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                 odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  yaw_ = tf::getYaw(odom->pose.pose.orientation);
  yaw_dot_ = odom->twist.twist.angular.z;

  last_odom_t_ = ros::Time::now();
  pose_updated_ = true;
}

// TODO: receive explore goal. recive map goal

void Nav2D::receiveGetMapGoal(const ddk_nav_2d::GetFirstMapGoal::ConstPtr &goal) {
  ROS_INFO("Nav2D: Received get first map goal.");
  Rate loop_rate(2);

  if (node_status_ != NAV_ST_IDLE) {
    ROS_WARN("Navigator is busy!");
    get_first_map_action_server_->setAborted();
    return;
  }

  if (get_first_map_action_server_->isPreemptRequested()) {
    ROS_INFO("Get first map has been preempted externally.");
    get_first_map_action_server_->setPreempted();
    return;
  }

  ddk_nav_2d::GetFirstMapFeedback f;

  // Go straight
  float distance = 0;
  while (true) {
    if (distance >= 2)
      break;
    get_first_map_action_server_->publishFeedback(f);
    if (line_tracker_Status_ == NAV_ST_IDLE) {
      goTo(1, 0, 0, 0, 0.0f, 0.0f, true);
      distance += 1;
    }
    spinOnce();
    loop_rate.sleep();
  }

  // Turn 360, get first map
  float rotation = 0;
  while (true) {
    if (rotation >= 6.282)
      break;
    // ROS_INFO("In the loop.");
    get_first_map_action_server_->publishFeedback(f);
    if (line_tracker_Status_ == NAV_ST_IDLE) {
      ROS_INFO("Move");
      goTo(0, 0, 0, 2.094, 0.0f, 0.0f, true);
      rotation += 2.094;
    }
    spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Set succeeded");
  get_first_map_action_server_->setSucceeded();
}

void Nav2D::receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal) {
  ROS_INFO("Nav2D: Received explore goal.");
  if (node_status_ != NAV_ST_IDLE) {
    ROS_WARN("Navigator is busy!");
    explore_action_server_->setAborted();
    return;
  }
  node_status_ = NAV_ST_EXPLORING;
  unsigned int cycle = 0;
  unsigned int last_check = 0;
  unsigned int recheck_cycles = min_replanning_period_ * frequency_;
  unsigned int recheck_threshold = max_replanning_period_ * frequency_;

  // Move to exploration target
  // Rate loopRate(mFrequency);
  Rate loop_rate(1);
  while (true) {

    if (explore_action_server_->isPreemptRequested()) {
      ROS_INFO("Exploration has been preempted externally.");
      explore_action_server_->setPreempted();
      return;
    }

    // get current map index
    if (!getMapIndex()) {
      ROS_ERROR("Exploration failed, could not get current position.");
      explore_action_server_->setAborted();
      return;
    }

    // recheck for exploration target
    bool recheck = last_check == 0 ||(recheck_cycles && (cycle - last_check > recheck_cycles));
    bool goal_reached = false;
    if (line_tracker_Status_ == NAV_ST_IDLE) {
      goal_reached = true;
      ROS_INFO("Set goal reached true. can continue explore");
    }

    if (recheck || goal_reached) {
      // WallTime startTime = WallTime::now();
      last_check = cycle;
      if (preparePlan()) {
        // ROS_INFO("Enter prepare plan true");
        int result = exploration_planner_->findExplorationTarget(&current_map_, start_point_, goal_point_);
        ROS_INFO("result: %d", result);
        switch (result) {
        case EXPL_TARGET_SET:
          ROS_INFO("EXPL TARGET SET");
          if (line_tracker_Status_ == NAV_ST_IDLE) {
            unsigned int goal_x = 0, goal_y = 0;
            if (current_map_.getCoordinates(goal_x, goal_y, goal_point_)) {
              float map_goal_x = current_map_.getOriginX() + (((double)goal_x + 0.5) * current_map_.getResolution());
              float map_goal_y = current_map_.getOriginY() + (((double)goal_y + 0.5) * current_map_.getResolution());
              float map_goal_z = 0.4;
              float yaw;

              yaw = atan2(pos_(1) - map_goal_y, pos_(0) - map_goal_x);
              yaw += PI;
              if (yaw < -PI) yaw += 2 * PI;
              if (yaw > PI) yaw -= 2 * PI;

              line_tracker_Status_ = NAV_ST_TURNING;

              // Line Tracker code
              // // Turn head
              // while(true){
              //     if (lineTrackerStatus == NAV_ST_IDLE) break;
              //     ROS_INFO("turning");
              //     if (lineTrackerStatus == NAV_ST_TURNING){
              //         goTo(pos_(0), pos_(1), pos_(2), yaw, 0.0f, 0.0f, false);
              //     }
              //     spinOnce();
              //     loopRate.sleep();
              // }
              // mStatus = NAV_ST_EXPLORING;
              //  // Go straight
              // goTo(worldGoalX, worldGoalY, worldGoalZ, yaw, 0.0f, 0.0f, false);

              ROS_INFO("goal: %f, %f, %f, %f", map_goal_x, map_goal_y, map_goal_z, yaw);
              double distance = std::pow(map_goal_x - pos_(0), 2) +
                                std::pow(map_goal_y - pos_(1), 2) +
                                std::pow(map_goal_z - pos_(2), 2);
              ROS_INFO("Distance to goal: %f", distance);
              node_status_ = NAV_ST_EXPLORING;

              double local_time = 0.0;

              // get current odom transform
              tf::StampedTransform transform;
              try {
                tf_Listener_.lookupTransform(map_frame_, robot_frame_, Time(0), transform);
              } catch (tf::TransformException &ex) {
                ROS_INFO("Couldn't get current odom transform");
                ROS_WARN("%s", ex.what());
                return;
              }

              Eigen::Affine3d dT_o_w;
              tf::transformTFToEigen(transform, dT_o_w);
              Eigen::Affine3f tf_odom_to_world = dT_o_w.cast<float>();

              geometry_msgs::Quaternion goal_quaternion = tf::createQuaternionMsgFromYaw(yaw);
              geometry_msgs::PoseStamped goal;
              goal.header.frame_id = map_frame_;
              goal.header.stamp = ros::Time::now();
              goal.pose.position.x = map_goal_x;
              goal.pose.position.y = map_goal_y;
              goal.pose.position.z = map_goal_z;
              goal.pose.orientation = goal_quaternion;

              bool ret = getJpsTraj(local_time, tf_odom_to_world, goal);
              if (ret) {
                ROS_INFO("get jps traj return true");
              }
              if (!ret)
                ROS_INFO("get jps return false");
            }
          }
          break;

        case EXPL_FINISHED: {
          ddk_nav_2d::ExploreResult r;
          r.final_pose.x = pos_(0);
          r.final_pose.y = pos_(1);
          r.final_pose.theta = yaw_;
          explore_action_server_->setSucceeded(r);
        }
          ROS_INFO("Exploration has finished.");
          return;

        case EXPL_WAITING:
          node_status_ = NAV_ST_WAITING;
          ROS_INFO("Exploration is waiting.");
          break;
        case EXPL_FAILED:
          break;
        default:
          ROS_ERROR("Exploration planner returned invalid status code: %d!", result);
        }
      }

      if (node_status_ == NAV_ST_EXPLORING) {
        if (line_tracker_Status_ == NAV_ST_NAVIGATING) {
          ROS_INFO("moving.");
          // WallTime endTime = WallTime::now();
        } else {
          explore_action_server_->setAborted();
          ROS_WARN("Exploration has failed!");
          node_status_ = NAV_ST_IDLE;
          return;
        }
      }
    }

    if (node_status_ == NAV_ST_EXPLORING) {
      // Publish feedback via ActionServer
      if (cycle % 10 == 0) {
        ddk_nav_2d::ExploreFeedback fb;
        fb.robot_pose.x = pos_(0);
        fb.robot_pose.y = pos_(1);
        fb.robot_pose.theta = yaw_;
        explore_action_server_->publishFeedback(fb);
      }
    }

    // Sleep remaining time
    cycle++;
    spinOnce();
    loop_rate.sleep();
  }
}

bool Nav2D::goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative) {
  kr_tracker_msgs::LineTrackerGoal goal;
  goal.x = x;
  goal.y = y;

  goal.relative = relative;
  // Convert relative translation in body frame to global frame
  if (relative) {
    goal.x = x * std::cos(yaw_) - y * std::sin(yaw_);
    goal.y = x * std::sin(yaw_) + y * std::cos(yaw_);
  }

  goal.z = z;
  goal.yaw = yaw;
  goal.v_des = v_des;
  goal.a_des = a_des;

  line_tracker_min_jerk_client_->sendGoal(
      goal, boost::bind(&Nav2D::trackerDoneCB, this, _1, _2),
      ClientType::SimpleActiveCallback(), ClientType::SimpleFeedbackCallback());
  line_tracker_Status_ = NAV_ST_NAVIGATING;

  return transition(line_tracker_min_jerk_);
}

bool Nav2D::trackPath(nav_msgs::Path planned_path) {
  kr_replanning_msgs::TrackPathGoal pathGoal;
  pathGoal.path = planned_path;
  track_path_action_client_->sendGoal(
      pathGoal, boost::bind(&Nav2D::trackPathDoneCB, this, _1, _2),
      trackPathClientType::SimpleActiveCallback(),
      trackPathClientType::SimpleFeedbackCallback());
  track_path_status_ = NAV_ST_NAVIGATING;
  return true;
}

void Nav2D::trackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result) {
  ROS_INFO("Goal reached.");
  line_tracker_Status_ = NAV_ST_IDLE;
}

void Nav2D::trackPathDoneCB(
    const actionlib::SimpleClientGoalState &state,
    const kr_replanning_msgs::TrackPathResultConstPtr &result) {
  ROS_INFO("Track Path done.");
}

bool Nav2D::transition(const std::string &tracker_str) {
  kr_tracker_msgs::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd) && transition_cmd.response.success) {
    active_tracker_ = tracker_str;
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }
  return false;
}

bool Nav2D::getMapIndex() {
  tf::StampedTransform transform;
  try {
    tf_Listener_.lookupTransform(map_frame_, robot_frame_, Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Could not get robot position: %s", ex.what());
    return false;
  }
  double world_x = transform.getOrigin().x();
  double world_y = transform.getOrigin().y();
  double world_theta = getYaw(transform.getRotation());

  unsigned int current_x = (world_x - current_map_.getOriginX()) / current_map_.getResolution();
  unsigned int current_y = (world_y - current_map_.getOriginY()) / current_map_.getResolution();
  unsigned int start_point;

  current_map_.getIndex(current_x, current_y, start_point);
  // signed char valueHere = current_map_.getData(current_x, current_y);
  // ROS_INFO("Check current position map value: %u", valueHere);
  start_point_ = start_point;

  ROS_INFO("Get map index: %f, %f, %d, %d, %d", world_x, world_y, current_x, current_y, start_point);
  return true;
}

bool Nav2D::preparePlan() {
  // Where am I?
  if (!getMapIndex()) return false;

  // Clear robot footprint in map
  unsigned int x = 0, y = 0;
  if (current_map_.getCoordinates(x, y, start_point_))
    for (int i = -cell_robot_radius_; i < (int)cell_robot_radius_; i++)
      for (int j = -cell_robot_radius_; j < (int)cell_robot_radius_; j++)
        current_map_.setData(x + i, y + j, 0);
  ROS_INFO("prepare plan, set Startpoint as %d", start_point_);
  return true;
}

bool Nav2D::getJpsTraj(const double &traj_time, const Eigen::Affine3f &o_w_transform, geometry_msgs::PoseStamped &min_cost_pt) {
  nav_msgs::GetPlan jps_srv;
  jps_srv.request.start = min_cost_pt;
  jps_srv.request.start.pose.position.x = o_w_transform.translation()[0];
  jps_srv.request.start.pose.position.y = o_w_transform.translation()[1];
  jps_srv.request.start.pose.position.z = o_w_transform.translation()[2];
  jps_srv.request.goal = min_cost_pt;
  if (jps_service_client_.call(jps_srv)) {
    ROS_INFO("Successful jps service call");

    if (jps_srv.response.plan.poses.size() == 0) {
      ROS_ERROR("JPS did not return a path");
      return false;
    }
    trackPath(jps_srv.response.plan);
    return true;
  } else {
    ROS_ERROR("Failed to jps call service");
    return false;
  }
}
