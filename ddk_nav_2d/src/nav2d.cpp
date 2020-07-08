#include <ddk_nav_2d/nav2d.h>
#include <string>

#include <math.h>

#define PI 3.14159265

Nav2D::Nav2D() {

  pnh_ = ros::NodeHandle("~");

  // Subscriber
  map_subscriber_ = nh_.subscribe("projected_map", 5, &Nav2D::mapSubscriberCB, this);
  pose_subscriber_ = nh_.subscribe("ground_truth/odom", 10, &Nav2D::poseSubscriberCB, this);

  // Publisher
  goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("navigator/goal_point", 5);

  // Load planer
  pnh_.param("min_replanning_period", min_replanning_period_, 8.0);
  pnh_.param("max_replanning_period", max_replanning_period_, 1.0);
  pnh_.param("exploration_strategy", exploration_strategy_, std::string("NearestFrontierPlanner"));
  try {
    plan_loader_ptr_.reset(new PlanLoader("nav2d_navigator", "ExplorationPlanner"));
    exploration_planner_ = plan_loader_ptr_->createInstance(exploration_strategy_);
    ROS_INFO("Successfully loaded exploration strategy [%s].", exploration_strategy_.c_str());

  } catch (pluginlib::PluginlibException &ex) {
    ROS_ERROR("Failed to load exploration plugin! Error: %s", ex.what());
    explore_action_server_ptr_ = NULL;
    plan_loader_ptr_ = NULL;
  }

  // Explore action server
  pnh_.param("explore_action_topic", explore_action_topic_, std::string(NAV_EXPLORE_ACTION));
  explore_action_server_ptr_.reset(new ExploreServerType(explore_action_topic_, boost::bind(&Nav2D::receiveExploreGoal, this, _1), false));
  explore_action_server_ptr_->start();


  node_status_ = NAV_ST_IDLE;
  line_tracker_status_ = NAV_ST_IDLE;
  line_tracker_min_jerk_ = "kr_trackers/LineTrackerMinJerk";
  line_tracker_min_jerk_client_ptr_.reset(new LineClientType(nh_, "trackers_manager/line_tracker_min_jerk/LineTracker", true));
  float server_wait_timeout;
  pnh_.param("Server_wait_timeout", server_wait_timeout, 0.5f);
  if (!line_tracker_min_jerk_client_ptr_->waitForServer(ros::Duration(server_wait_timeout))) {
    ROS_ERROR("LineTrackerMinJerk server not found.");
  }

  traj_tracker_status_ = NAV_ST_IDLE;
  traj_tracker_ = "kr_trackers/TrajectoryTracker";
  traj_tracker_client_ptr_.reset(new TrajectoryClientType(nh_, "trackers_manager/trajectory_tracker/TrajectoryTracker", true));
  if (!traj_tracker_client_ptr_->waitForServer(ros::Duration(server_wait_timeout))) {
    ROS_ERROR("TrajectoryTracker server not found.");
  }

  // Services
  active_tracker_ = "";
  srv_transition_ = nh_.serviceClient<kr_tracker_msgs::Transition>("trackers_manager/transition");

  // tf listener
  pnh_.param("map_frame", map_frame_, std::string("ddk/odom"));
  pnh_.param("robot_frame", robot_frame_, std::string("ddk/base_link"));
  robot_frame_ = tf_listener_.resolve(robot_frame_);
  map_frame_ = tf_listener_.resolve(map_frame_);

  // Planning related param
  pnh_.param("robot_radius", robot_radius_, 0.2);
  cost_lethal_ = 1;
  map_updated_ = false;

  // replanning
  frequency_ = 1;
  jps_service_client_ = nh_.serviceClient<nav_msgs::GetPlan>("jps_plan_service");

  track_path_status_ = NAV_ST_IDLE;
  track_path_action_client_ptr_.reset(new TrackPathClientType(nh_, "TrackPathAction", true));
  // track_path_action_client_ = new trackPathClientType(nh_, "ddk/TrackPathAction", true);
  if (!track_path_action_client_ptr_->waitForServer(ros::Duration(server_wait_timeout))) {
    ROS_ERROR("track path action server not found.");
  }
}


Nav2D::~Nav2D() {
  ROS_INFO("Clean.");
  exploration_planner_.reset();
}


void Nav2D::mapSubscriberCB(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  // ROS_INFO("Map update received.");
  current_map_.update(*map);

  if (map_updated_ == false) {
    ROS_INFO("Navigator is now initialized.");
    cell_robot_radius_ = 1;
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


void Nav2D::receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal) {
  ROS_INFO("Nav2D: Received explore goal.");
  if (node_status_ != NAV_ST_IDLE) {
    ROS_WARN("Navigator is busy!");
    explore_action_server_ptr_->setAborted();
    return;
  }
  node_status_ = NAV_ST_EXPLORING;
  unsigned int cycle = 0;
  unsigned int last_check = 0;
  unsigned int recheck_cycles = min_replanning_period_ * frequency_;
  // unsigned int recheck_threshold = max_replanning_period_ * frequency_;
  bool moving = false;
  float last_goal_x, last_goal_y;

  bool track_path = false;
  bool recheck_goal = false;
  bool recheck;
  ros::Rate loop_rate(2);

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
    // ROS_INFO("In the loop.");
    if (line_tracker_status_ == NAV_ST_IDLE) {
      // ROS_INFO("Move");
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

    if (track_path_status_ == NAV_ST_IDLE && traj_tracker_status_ == NAV_ST_IDLE && line_tracker_status_ == NAV_ST_IDLE){
      moving = false;
    } else {
      moving = true;
    }

    // recheck for exploration target
    if (recheck_goal) {
      recheck = last_check == 0 ||(recheck_cycles && (cycle - last_check > recheck_cycles));
    } else {
      recheck = recheck_goal;
    }

    // No matter what cases, need enter the loop.
    if (!moving || recheck){
      if (preparePlan()) {
        // ROS_INFO("Enter prepare plan true");
        int result = exploration_planner_->findExplorationTarget(&current_map_, start_point_, goal_point_);
        // ROS_INFO("result: %d", result);

        switch (result) {
        case EXPL_TARGET_SET:
          ROS_INFO("EXPL TARGET SET");
          if (recheck || !moving) {
            
            unsigned int goal_x = 0, goal_y = 0;
            if (current_map_.getCoordinates(goal_x, goal_y, goal_point_)) {
              float map_goal_x = current_map_.getOriginX() + (((double)goal_x + 0.5) * current_map_.getResolution());
              float map_goal_y = current_map_.getOriginY() + (((double)goal_y + 0.5) * current_map_.getResolution());
              float map_goal_z = 0.4;

              if (recheck && moving){
              last_check = cycle;
              ROS_INFO("Enter recheck exploration goal point.");
                if (std::abs(last_goal_x - map_goal_x) > 0.1 || std::abs(last_goal_y - map_goal_y) > 0.1){
                  // should cancel current move
                  cancelCurrentGoal();
                  ROS_INFO("Cancel current goal. reason: x: %f y: %f", std::abs(last_goal_x - map_goal_x), std::abs(last_goal_y - map_goal_y) > 0.1);
                  // getMapIndex();
                  break;
                }
              }
              moving = true;

              last_goal_x = map_goal_x;
              last_goal_y = map_goal_y;

              // float map_goal_x = 1.0;
              // float map_goal_y = 0.0;
              // float map_goal_z = 0.6;
              float yaw;

              yaw = atan2(pos_(1) - map_goal_y, pos_(0) - map_goal_x);
              yaw += PI;
              if (yaw < -PI) yaw += 2 * PI;
              if (yaw > PI) yaw -= 2 * PI;

              if (!track_path){
                // Turn head, needed for using Trajectory tracker.
                line_tracker_status_ = NAV_ST_TURNING;
                while(true){
                    if (line_tracker_status_ == NAV_ST_IDLE) break;
                    // ROS_INFO("turning");
                    if (line_tracker_status_ == NAV_ST_TURNING){
                        goTo(pos_(0), pos_(1), pos_(2), yaw, 0.0f, 0.0f, false);
                    }
                    ros::spinOnce();
                    loop_rate.sleep();
                }
              }
              //  // Go straight
              // goTo(worldGoalX, worldGoalY, worldGoalZ, yaw, 0.0f, 0.0f, false);

              ROS_INFO("goal: %f, %f, %f, %f", map_goal_x, map_goal_y, map_goal_z, yaw);
              // Check goal occupancy:
              signed char goal_value = current_map_.getData(goal_point_);
              ROS_INFO("Current Goal Value is: %u", goal_value);
              double distance = std::pow(map_goal_x - pos_(0), 2) +
                                std::pow(map_goal_y - pos_(1), 2) +
                                std::pow(map_goal_z - pos_(2), 2);
              ROS_INFO("Distance to goal: %f", distance);
              node_status_ = NAV_ST_EXPLORING;

              double local_time = 0.0;

              // get current odom transform
              tf::StampedTransform transform;
              try {
                tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
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

              goal_publisher_.publish(goal);

              bool ret = getJpsTraj(local_time, tf_odom_to_world, goal, track_path);
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
          explore_action_server_ptr_->setSucceeded(r);
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
      } else {
        explore_action_server_ptr_->setAborted();
        ROS_WARN("Exploration has failed!");
        node_status_ = NAV_ST_IDLE;
        return;
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
}


void Nav2D::lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result) {
  ROS_INFO("Goal reached.");
  line_tracker_status_ = NAV_ST_IDLE;
}


void Nav2D::trackPathDoneCB(
    const actionlib::SimpleClientGoalState &state,
    const kr_replanning_msgs::TrackPathResultConstPtr &result) {
  ROS_INFO("Track Path done.");
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
    active_tracker_ = tracker_str;
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }
  return false;
}



bool Nav2D::getMapIndex() {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
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
