#include <ddk_nav_2d/nav2d.h>
#include <string>

#include <math.h>

#define PI 3.14159265

Nav2D::Nav2D() {

  pnh_ = ros::NodeHandle("~");

  // Params
  pnh_.param<double>("min_recheck_period", min_recheck_period_, 8.0);
  pnh_.param<double>("server_wait_timeout", server_wait_timeout_, 3.0);
  pnh_.param<std::string>("map_frame", map_frame_, std::string("ddk/odom"));
  pnh_.param<std::string>("robot_frame", robot_frame_, std::string("ddk/base_link"));
  pnh_.param<int>("cell_robot_radius", cell_robot_radius_, 1);
  pnh_.param<double>("frequency", frequency_, 2.0);
  pnh_.param<int>("occupied_cell_threshold", occupied_cell_threshold_, 1);
  pnh_.param<std::string>("explore_action_topic", explore_action_topic_, std::string(NAV_EXPLORE_ACTION));
  pnh_.param<double>("map_inflation_radius", map_inflation_radius_, 0.5);
  pnh_.param<float>("flight_height", flight_height_, 1.2);
  pnh_.param<bool>("goal_recheck", goal_recheck_, true);
  pnh_.param<double>("obstacle_scan_range", obstacle_scan_range_, 1.0);
  pnh_.param<int>("goal_frontier_num_threshold", goal_frontier_threshold_, 30);
  pnh_.param<double>("frontier_distance_threshold", frontier_distance_threshold_, 0.5);

  // Subscriber
  map_subscriber_ = nh_.subscribe("projected_map", 5, &Nav2D::mapSubscriberCB, this);
  pose_subscriber_ = nh_.subscribe("odom", 10, &Nav2D::poseSubscriberCB, this);
  // Publisher
  goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_point", 5);

  //for debug usage
  inflated_map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("inflated_map", 5);

  // Services
  srv_transition_ = nh_.serviceClient<kr_tracker_msgs::Transition>("trackers_manager/transition");
  jps_service_client_ = nh_.serviceClient<nav_msgs::GetPlan>("jps_plan_service");

  // Action client
  line_tracker_min_jerk_client_ptr_.reset(new LineClientType(nh_, "trackers_manager/line_tracker_min_jerk/LineTracker", true));
  if (!line_tracker_min_jerk_client_ptr_->waitForServer(ros::Duration(server_wait_timeout_))) {
    ROS_ERROR("LineTrackerMinJerk server not found.");
  }
  traj_tracker_client_ptr_.reset(new TrajectoryClientType(nh_, "trackers_manager/trajectory_tracker/TrajectoryTracker", true));
  if (!traj_tracker_client_ptr_->waitForServer(ros::Duration(server_wait_timeout_))) {
    ROS_ERROR("TrajectoryTracker server not found.");
  }
  track_path_action_client_ptr_.reset(new TrackPathClientType(nh_, "TrackPathAction", true));
  if (!track_path_action_client_ptr_->waitForServer(ros::Duration(server_wait_timeout_))) {
    ROS_ERROR("track path action server not found.");
  }

  // tf listener
  tf_listener_ptr_.reset(new tf2_ros::TransformListener(tfBuffer_));
  // robot_frame_ = tf_listener_.resolve(robot_frame_);
  // map_frame_ = tf_listener_.resolve(map_frame_);

  // Init Params
  map_updated_ = false;
  node_status_ = NAV_ST_IDLE;
  line_tracker_status_ = NAV_ST_IDLE;
  track_path_status_ = NAV_ST_IDLE;
  traj_tracker_status_ = NAV_ST_IDLE;

  // Strings for tracker transition
  line_tracker_min_jerk_ = "kr_trackers/LineTrackerMinJerk";
  traj_tracker_ = "kr_trackers/TrajectoryTracker";

  frontier_planner_.setObstacleScanRange(obstacle_scan_range_);
  frontier_planner_.setGoalFrontierThreshold(goal_frontier_threshold_);
  frontier_planner_.setFrontierDistanceThreshold(frontier_distance_threshold_);

  // Action server
  explore_action_server_ptr_.reset(new ExploreServerType(explore_action_topic_, boost::bind(&Nav2D::receiveExploreGoal, this, _1), false));
  explore_action_server_ptr_->start();
}


Nav2D::~Nav2D() {
  // exploration_planner_.reset();
}


void Nav2D::mapSubscriberCB(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  boost::mutex::scoped_lock lock(map_mutex_);
  current_map_.update(*map);
  inflated_map_.update(*map);
  if (map_updated_ == false) {
    ROS_INFO("Navigator is now initialized.");
    cell_map_inflation_radius_ = map_inflation_radius_ / current_map_.getResolution();
    ROS_INFO("Inflation cell radius: %u", cell_map_inflation_radius_);
    map_inflation_tool_.computeCaches(cell_map_inflation_radius_);
    current_map_.setLethalCost((signed char)occupied_cell_threshold_);
    inflated_map_.setLethalCost((signed char)occupied_cell_threshold_);
  }
  map_updated_ = true;
  inflated_map_inflated_ = false;

  // Compute map inflation
  if (!inflated_map_inflated_){
    map_inflation_tool_.inflateMap(&inflated_map_);
    inflated_map_inflated_ = true;
    inflated_map_publisher_.publish(inflated_map_.getMap());
  }
  // Try grid map
  // bool status = grid_map::GridMapRosConverter::fromOccupancyGrid(*map, "test", map_test_);
  // grid_map::Position start(0.0, 0.0);
  // grid_map::Position goal(0.0, 0.0);
  // frontier_planner_.findExplorationTarget(&map_test_, start, goal);
  // if (status) ROS_INFO("Gridmap connected");
}


void Nav2D::poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom) {
  // ROS_INFO("pose update received.");
  pos_(0) = odom->pose.pose.position.x;
  pos_(1) = odom->pose.pose.position.y;
  pos_(2) = odom->pose.pose.position.z;

  odom_q_ = Quat(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                 odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  // yaw_ = tf2::getYaw(odom->pose.pose.orientation);
  double yaw, _pitch, _roll;
  tf2::Matrix3x3(tf2::Quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                                 odom->pose.pose.orientation.z, odom->pose.pose.orientation.w)).getEulerYPR(yaw, _pitch, _roll);
  yaw_ = yaw;
  last_odom_t_ = ros::Time::now();
}


void Nav2D::receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal) {
  ROS_INFO("Nav2D: Received explore goal.");
  if (!map_updated_) {
    ROS_ERROR("No map received!");
    explore_action_server_ptr_->setAborted();
    return;
  }

  if (node_status_ != NAV_ST_IDLE) {
    ROS_WARN("Navigator is busy!");
    explore_action_server_ptr_->setAborted();
    return;
  }

  node_status_ = NAV_ST_EXPLORING;

  // Params related to recheck goal
  unsigned int cycle = 0;
  unsigned int last_check = 0;
  bool recheck;
  unsigned int recheck_cycles = min_recheck_period_ * frequency_;
  // unsigned int recheck_cycles;
  float last_goal_x, last_goal_y;

  // set to change use traj tracker / TrackPath.
  bool track_path = true;

  bool moving = false;
  ros::Rate loop_rate(frequency_);

  // Go straight
  float straight = 0;
  while (true) {
    if (straight >= 0.5 && line_tracker_status_ == NAV_ST_IDLE)
      break;
    if (line_tracker_status_ == NAV_ST_IDLE) {
      goTo(0.5, 0, 0, 0, 0.0f, 0.0f, true);
      straight += 0.5;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Turn 360, get first map
  float rotation = 0;
  while (true) {
    if (rotation >= 6.282 && line_tracker_status_ == NAV_ST_IDLE)
      break;
    if (line_tracker_status_ == NAV_ST_IDLE) {
      goTo(0, 0, 0, 3.141, 0.0f, 0.0f, true);
      rotation += 3.141;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }


  // Move to exploration target
  while (true) {
    if (explore_action_server_ptr_->isPreemptRequested()) {
      ROS_INFO("Exploration has been preempted externally.");
      explore_action_server_ptr_->setPreempted();
      return;
    }

    // get current map index
    if (!getMapIndex()) {
      ROS_ERROR("Exploration failed, could not get current position.");
      explore_action_server_ptr_->setAborted();
      return;
    }

    // check current status
    if (track_path_status_ == NAV_ST_IDLE && traj_tracker_status_ == NAV_ST_IDLE && line_tracker_status_ == NAV_ST_IDLE){
      moving = false;
    } else {
      moving = true;
    }

    // recheck for exploration target
    if (goal_recheck_) {
      recheck = last_check == 0 ||(recheck_cycles && (cycle - last_check > recheck_cycles));
    } else {
      recheck = goal_recheck_;
    }

    // Not moving(reached goal) or recheck, need to find new frontiers.
    if (!moving || recheck){
      boost::mutex::scoped_lock lock(map_mutex_);
      if (preparePlan()) {
        int result = frontier_planner_.findExplorationTarget(&inflated_map_, start_point_, goal_point_);
        switch (result) {

        case EXPL_TARGET_SET:
          ROS_INFO("EXPL TARGET SET");
          if (recheck || !moving) {
            unsigned int goal_x = 0, goal_y = 0;
            if (inflated_map_.getCoordinates(goal_x, goal_y, goal_point_)) {
              float map_goal_x = inflated_map_.getOriginX() + (((double)goal_x + 0.5) * inflated_map_.getResolution());
              float map_goal_y = inflated_map_.getOriginY() + (((double)goal_y + 0.5) * inflated_map_.getResolution());
              float map_goal_z = flight_height_;

              if (recheck && moving){
              last_check = cycle;
              ROS_INFO("Enter recheck exploration goal point.");
                if (std::abs(last_goal_x - map_goal_x) > 0.1 || std::abs(last_goal_y - map_goal_y) > 0.1){
                  // should cancel current move
                  cancelCurrentGoal();
                  ROS_INFO("Cancel current goal. reason: x: %f y: %f", std::abs(last_goal_x - map_goal_x), std::abs(last_goal_y - map_goal_y));
                  // getMapIndex();
                  break;
                }
              }
              // Change current status
              moving = true;
              node_status_ = NAV_ST_EXPLORING;

              last_goal_x = map_goal_x;
              last_goal_y = map_goal_y;

              float yaw;
              yaw = atan2(pos_(1) - map_goal_y, pos_(0) - map_goal_x);
              yaw += PI;
              if (yaw < -PI) yaw += 2 * PI;
              if (yaw > PI) yaw -= 2 * PI;

              // Turn head, needed for Trajectory tracker.
              if (!track_path){
                line_tracker_status_ = NAV_ST_TURNING;
                while(true){
                    if (line_tracker_status_ == NAV_ST_IDLE) break;
                    if (line_tracker_status_ == NAV_ST_TURNING){
                        goTo(pos_(0), pos_(1), pos_(2), yaw, 0.0f, 0.0f, false);
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }
              }

              ROS_INFO("goal: %f, %f, %f, %f", map_goal_x, map_goal_y, map_goal_z, yaw);
              // Check goal occupancy:
              signed char goal_value = inflated_map_.getData(goal_point_);
              ROS_INFO("Current Goal Value is: %u", goal_value);
              double distance = std::sqrt(std::pow(map_goal_x - pos_(0), 2) +
                                std::pow(map_goal_y - pos_(1), 2) +
                                std::pow(map_goal_z - pos_(2), 2));
              ROS_INFO("Distance to goal: %f", distance);
              // TODO: check the hardcode according to spin speed and speed
              if (distance > 3) recheck_cycles = (3 * 5 + 5) * frequency_;
              recheck_cycles = (distance * 5 + 5) * frequency_;

              // Gen arguments needed for getJpsTraj function.
              double local_time = 0.0;
              // get current odom transform

              geometry_msgs::TransformStamped transformStamped;
              try {
                transformStamped = tfBuffer_.lookupTransform(map_frame_, robot_frame_, ros::Time(0));
              } 
              catch (tf2::TransformException &ex) {
                ROS_INFO("Couldn't get current odom transform");
                ROS_WARN("%s", ex.what());
                return;
              }
              Eigen::Affine3d dT_o_w;
              dT_o_w = tf2::transformToEigen(transformStamped);
              Eigen::Affine3f tf_odom_to_world = dT_o_w.cast<float>();

              // Check what is the expected goal yaw.
              double goal_yaw = getGoalHeading(goal_point_);
              if (goal_yaw < 0) ROS_ERROR("Goal heading -1.");
            
              tf2::Quaternion tf2_quaternion;
              tf2_quaternion.setRPY(0, 0, goal_yaw);
              geometry_msgs::Quaternion goal_quaternion = tf2::toMsg(tf2_quaternion);

              geometry_msgs::PoseStamped goal;
              goal.header.frame_id = map_frame_;
              goal.header.stamp = ros::Time::now();
              goal.pose.position.x = map_goal_x;
              goal.pose.position.y = map_goal_y;
              goal.pose.position.z = map_goal_z;
              goal.pose.orientation = goal_quaternion;

              // Publish goal for visualization in rviz
              goal_publisher_.publish(goal);

              bool ret = getJpsTraj(local_time, tf_odom_to_world, goal, track_path);

            }
          }
          break;

        case EXPL_FINISHED: {
          ddk_nav_2d::ExploreResult r;
          r.final_pose.x = pos_(0);
          r.final_pose.y = pos_(1);
          r.final_pose.theta = yaw_;
          explore_action_server_ptr_->setSucceeded(r);
        }
          ROS_INFO("Exploration has finished.");
          return;

        case EXPL_WAITING:
          node_status_ = NAV_ST_WAITING;
          ROS_INFO("Exploration is waiting.");
          break;
        case EXPL_FAILED:
          ROS_INFO("Exploration failed.");
          break;
        default:
          ROS_ERROR("Exploration planner returned invalid status code: %d!", result);
        }
      } else {
        // Prepare plan failed.
        ROS_ERROR("Exploration failed, prepare plan failed.");
        explore_action_server_ptr_->setAborted();
        return;
      }
    }


    if (node_status_ == NAV_ST_EXPLORING) {
      if (moving) {
        // ROS_INFO("moving.");
        // Publish feedback via ActionServer
        if (cycle % 10 == 0) {
          ddk_nav_2d::ExploreFeedback fb;
          fb.robot_pose.x = pos_(0);
          fb.robot_pose.y = pos_(1);
          fb.robot_pose.theta = yaw_;
          explore_action_server_ptr_->publishFeedback(fb);
        }
      }
    }

    // Sleep remaining time
    cycle++;
    ros::spinOnce();
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

  line_tracker_min_jerk_client_ptr_->sendGoal(
      goal, boost::bind(&Nav2D::lineTrackerDoneCB, this, _1, _2),
      LineClientType::SimpleActiveCallback(), LineClientType::SimpleFeedbackCallback());
  line_tracker_status_ = NAV_ST_NAVIGATING;
  return transition(line_tracker_min_jerk_);
}


bool Nav2D::trackPath(nav_msgs::Path planned_path, bool method) {
  if (method){
    kr_replanning_msgs::TrackPathGoal path_goal;
    path_goal.path = planned_path;
    track_path_action_client_ptr_->sendGoal(
      path_goal, boost::bind(&Nav2D::trackPathDoneCB, this, _1, _2),
      TrackPathClientType::SimpleActiveCallback(),
      TrackPathClientType::SimpleFeedbackCallback());
    track_path_status_ = NAV_ST_NAVIGATING;
    ROS_INFO("Send Track Path goal to server.");
    return true;
  } else {
    kr_tracker_msgs::TrajectoryTrackerGoal path_goal;
    for (int i = 0; i < planned_path.poses.size(); i++){
      path_goal.waypoints.push_back(planned_path.poses[i].pose);
    }
    traj_tracker_client_ptr_->sendGoal(
      path_goal, boost::bind(&Nav2D::trajTrackerDoneCB, this, _1, _2),
      TrajectoryClientType::SimpleActiveCallback(), TrajectoryClientType::SimpleFeedbackCallback());
    traj_tracker_status_ = NAV_ST_NAVIGATING;
    ROS_INFO("Send trajectory tracker goal to server.");
    return transition(traj_tracker_);
  }
  return true;
}


void Nav2D::cancelCurrentGoal() {
  if (track_path_status_ != NAV_ST_IDLE) {
    track_path_action_client_ptr_->cancelGoal();
    track_path_status_ = NAV_ST_IDLE;
  }
  if (traj_tracker_status_ != NAV_ST_IDLE) {
    traj_tracker_client_ptr_->cancelGoal();
    traj_tracker_status_ = NAV_ST_IDLE;
  }
  if (line_tracker_status_ != NAV_ST_IDLE) {
    line_tracker_min_jerk_client_ptr_->cancelGoal();
    line_tracker_status_ = NAV_ST_IDLE;
  }
  ROS_WARN("Cancel current goal");
}


void Nav2D::lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result) {
  ROS_INFO("Goal reached.");
  line_tracker_status_ = NAV_ST_IDLE;
}


void Nav2D::trackPathDoneCB(
    const actionlib::SimpleClientGoalState &state,
    const kr_replanning_msgs::TrackPathResultConstPtr &result) {
  ROS_WARN("Track Path done. results: %d, %s", result->result, result->error_msg);
  track_path_status_ = NAV_ST_IDLE;
}


void Nav2D::trajTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr &result) {
  ROS_INFO("trajTracker goal reached.");
  traj_tracker_status_ = NAV_ST_IDLE;
}


bool Nav2D::transition(const std::string &tracker_str) {
  kr_tracker_msgs::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd) && transition_cmd.response.success) {
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }
  return false;
}


bool Nav2D::getMapIndex() {
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tfBuffer_.lookupTransform(map_frame_, robot_frame_, ros::Time(0));
  } 
  catch (tf2::TransformException &ex) {
    ROS_INFO("Couldn't get robot position");
    ROS_WARN("%s", ex.what());
    return false;
  }
  tf2::Stamped<tf2::Transform> transform;
  tf2::fromMsg(transform_stamped, transform);

  double world_x = transform.getOrigin().x();
  double world_y = transform.getOrigin().y();
  unsigned int current_x = (world_x - inflated_map_.getOriginX()) / inflated_map_.getResolution();
  unsigned int current_y = (world_y - inflated_map_.getOriginY()) / inflated_map_.getResolution();

  unsigned int start_point;
  inflated_map_.getIndex(current_x, current_y, start_point);
  start_point_ = start_point;

  // ROS_INFO("Get current map index: worldX:%f, worldY:%f, mapInd_x:%d, mapInd_y:%d, startpoint:%d", world_x, world_y, current_x, current_y, start_point);
  return true;
}


bool Nav2D::preparePlan() {
  if (!getMapIndex()) return false;

  // Clear robot footprint in map
  unsigned int x = 0, y = 0;
  if (inflated_map_.getCoordinates(x, y, start_point_))
    for (int i = -cell_robot_radius_; i < cell_robot_radius_; i++)
      for (int j = -cell_robot_radius_; j < cell_robot_radius_; j++)
        inflated_map_.setData(x + i, y + j, 0);

  inflated_map_publisher_.publish(inflated_map_.getMap());
  ROS_INFO("prepare plan ready, set Startpoint as %d", start_point_);
  return true;
}


bool Nav2D::getJpsTraj(const double &traj_time, const Eigen::Affine3f &o_w_transform, geometry_msgs::PoseStamped &min_cost_pt, bool method) {
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
    trackPath(jps_srv.response.plan, method);
    return true;
  } else {
    ROS_ERROR("Failed to jps call service");
    return false;
  }
}

double Nav2D::getGoalHeading(unsigned int goal_index) {
  unsigned int map_width = inflated_map_.getWidth();
  int y = goal_index / map_width;
  int x = goal_index % map_width;
  if(inflated_map_.getData(x-1, y-1) == -1) return 225*PI/180;
  if(inflated_map_.getData(x-1, y  ) == -1) return 180*PI/180;
  if(inflated_map_.getData(x-1, y+1) == -1) return 135*PI/180;
  if(inflated_map_.getData(x  , y-1) == -1) return 270*PI/180;
  if(inflated_map_.getData(x  , y+1) == -1) return 90*PI/180;
  if(inflated_map_.getData(x+1, y-1) == -1) return 315*PI/180;
  if(inflated_map_.getData(x+1, y  ) == -1) return 0.0;
  if(inflated_map_.getData(x+1, y+1) == -1) return 45*PI/180;

  return -1.0;
}
