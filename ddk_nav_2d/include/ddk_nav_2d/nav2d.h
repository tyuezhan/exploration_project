#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ddk_nav_2d/GetFirstMapAction.h>
#include <ddk_nav_2d/GridMap.h>
#include <ddk_nav_2d/MapInflationTool.h>
#include <ddk_nav_2d/commands.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

// include action
#include <ddk_nav_2d/ExploreAction.h>
#include <ddk_nav_2d/GetFirstMapAction.h>

// TODO: Delete unnecessary part in Exploration planner
#include <ddk_nav_2d/ExplorationPlanner.h>

#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrajectoryTrackerAction.h>
#include <kr_tracker_msgs/Transition.h>

#include <ddk_nav_2d/ddkPlanner.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <kr_replanning_msgs/TrackPathAction.h>

class Nav2D {

public:
  Nav2D();
  ~Nav2D();

  typedef Eigen::Vector3f Vec3;
  typedef Eigen::Quaternionf Quat;

  Vec3 pos() { return pos_; }
  Vec3 vel() { return vel_; }
  float yaw() { return yaw_; }

  // Subscriber call back function
  void mapSubscriberCB(const nav_msgs::OccupancyGrid::ConstPtr &map);
  void poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom);

  // Action Server goal callback
  void receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal);
  void receiveGetMapGoal(const ddk_nav_2d::GetFirstMapGoal::ConstPtr &goal);

  // Line tracker goto
  bool goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative);
  
  // Tracker transition
  bool transition(const std::string &tracker_str);
  
  // Tracker done callback
  void lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result);
  void trajTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr &result);

  // 2D exploration
  bool getMapIndex();
  bool preparePlan();
  void cancelCurrentGoal();

  // local replanning
  bool getJpsTraj(const double &traj_time, const Eigen::Affine3f &o_w_transform, geometry_msgs::PoseStamped &min_cost_pt, bool method);
  
  //method == True will use trackPath action. False will use TrajectoryTracker
  bool trackPath(nav_msgs::Path planned_path, bool method);   
  void trackPathDoneCB(const actionlib::SimpleClientGoalState &state, const kr_replanning_msgs::TrackPathResultConstPtr &result);

  // 3D exploration
  ddkPlanner mPlanner;

private:
  typedef actionlib::SimpleActionServer<ddk_nav_2d::GetFirstMapAction> GetFirstMapActionServer;
  typedef actionlib::SimpleActionServer<ddk_nav_2d::ExploreAction> ExploreActionServer;
  typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> LineClientType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::TrajectoryTrackerAction> TrajectoryClientType;
  typedef actionlib::SimpleActionClient<kr_replanning_msgs::TrackPathAction> trackPathClientType;

  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber map_subscriber_;
  ros::Subscriber pose_subscriber_;

  // Publisher
  ros::Publisher goal_publisher_;

  // Status param
  int node_status_;
  int line_tracker_status_;
  int track_path_status_;
  int traj_tracker_status_;

  //  Pose param
  Vec3 pos_, vel_;
  float yaw_, yaw_dot_;
  Quat odom_q_, imu_q_;
  ros::Time last_odom_t_;

  // Map related things
  // MapInflationTool inflation_tool_;

  // 2D map related param
  bool map_updated_;
  bool pose_updated_;
  GridMap current_map_;

  // TF param
  tf::TransformListener tf_Listener_;
  std::string map_frame_;
  std::string robot_frame_;

  // 2D frontier exploration related param
  std::string exploration_strategy_;
  boost::shared_ptr<ExplorationPlanner> exploration_planner_;
  PlanLoader *plan_loader_;

  double min_replanning_period_;
  double max_replanning_period_;
  unsigned int goal_point_;
  unsigned int start_point_;
  double frequency_;
  double inflation_radius_;
  double robot_radius_;
  unsigned int cell_inflation_radius_;
  unsigned int cell_robot_radius_;
  signed char cost_obstacle_;
  signed char cost_lethal_;


  // action Server and param:  get first map server; exploration server
  GetFirstMapActionServer *get_first_map_action_server_;
  ExploreActionServer *explore_action_server_;

  std::string explore_action_topic_;
  std::string get_map_action_topic_;

  // action Client: line tracker; trajectory tracker 
  LineClientType *line_tracker_min_jerk_client_;
  TrajectoryClientType *traj_tracker_client_;
  std::string line_tracker_min_jerk_;
  std::string traj_tracker_;

  // Services client and param: tracker transition; jps planner
  ros::ServiceClient srv_transition_;
  std::string active_tracker_;

  ros::ServiceClient jps_service_client_;
  trackPathClientType *track_path_action_client_;
};