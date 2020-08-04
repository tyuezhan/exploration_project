#include <ros/ros.h>
#include <std_srvs/Trigger.h>
// #include <pluginlib/class_loader.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <kr_replanning_msgs/TrackPathAction.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>

#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrajectoryTrackerAction.h>
#include <kr_tracker_msgs/Transition.h>

#include <ddk_nav_2d/GridMap.h>
#include <ddk_nav_2d/commands.h>
#include <ddk_nav_2d/MapInflationTool.h>
#include <ddk_nav_2d/ExploreAction.h>
#include <ddk_nav_2d/ExplorationPlanner.h>

// #include <grid_map_ros/grid_map_ros.hpp>

#include <ddk_nav_2d/frontier_planner.h>

class Nav2D {

public:
  Nav2D();
  ~Nav2D();

  typedef Eigen::Vector3f Vec3;
  typedef Eigen::Quaternionf Quat;

  Vec3 pos() { return pos_; }
  float yaw() { return yaw_; }

  // Subscriber call back function
  void mapSubscriberCB(const nav_msgs::OccupancyGrid::ConstPtr &map);
  void poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom);

  // Action Server goal callback
  void receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal);

  // Moving the robot: goTo for line tracker min jerk. trackPath for traj tracker/TrackPath action
  bool goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative);
    //method == True will use trackPath action. False will use TrajectoryTracker
  bool trackPath(nav_msgs::Path planned_path, bool method);   

  // Tracker transition
  bool transition(const std::string &tracker_str);
  
  // action client done callback
  void lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result);
  void trajTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::TrajectoryTrackerResultConstPtr &result);
  void trackPathDoneCB(const actionlib::SimpleClientGoalState &state, const kr_replanning_msgs::TrackPathResultConstPtr &result);

  // 2D exploration
  bool getMapIndex();
  bool preparePlan();
  void cancelCurrentGoal();
  double getGoalHeading(unsigned int goal_index);

  // replanning
  bool getJpsTraj(const double &traj_time, const Eigen::Affine3f &o_w_transform, geometry_msgs::PoseStamped &min_cost_pt, bool method);


private:
  // typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;
  typedef actionlib::SimpleActionServer<ddk_nav_2d::ExploreAction> ExploreServerType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> LineClientType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::TrajectoryTrackerAction> TrajectoryClientType;
  typedef actionlib::SimpleActionClient<kr_replanning_msgs::TrackPathAction> TrackPathClientType;

  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber map_subscriber_;
  ros::Subscriber pose_subscriber_;
  // Publisher
  ros::Publisher goal_publisher_;

  ros::Publisher inflated_map_publisher_;

  // Status param
  int node_status_;
  int line_tracker_status_;
  int track_path_status_;
  int traj_tracker_status_;

  //  Map & Pose
  // grid_map::GridMap map_test_;
  bool map_updated_;
  GridMap current_map_, inflated_map_;
  Vec3 pos_;
  float yaw_;
  Quat odom_q_;
  ros::Time last_odom_t_;
  MapInflationTool map_inflation_tool_;
  bool inflated_map_inflated_;
  float flight_height_;

  // TF param
  tf::TransformListener tf_listener_;
  std::string map_frame_;
  std::string robot_frame_;

  // 2D frontier exploration related param
  FrontierPlanner frontier_planner_;
  bool goal_recheck_;
  double obstacle_scan_range_;
  int goal_frontier_threshold_;

  double min_recheck_period_;
  unsigned int start_point_;
  unsigned int goal_point_;
  double frequency_;
  int cell_robot_radius_;
  int occupied_cell_threshold_;
  double map_inflation_radius_;
  unsigned int cell_map_inflation_radius_;

  // strings
  std::string explore_action_topic_;
  std::string line_tracker_min_jerk_;
  std::string traj_tracker_;

  // action Server and param:  exploration server
  std::unique_ptr<ExploreServerType> explore_action_server_ptr_;

  // action Client: line tracker; trajectory tracker; track path
  std::unique_ptr<LineClientType> line_tracker_min_jerk_client_ptr_;
  std::unique_ptr<TrajectoryClientType> traj_tracker_client_ptr_;
  std::unique_ptr<TrackPathClientType> track_path_action_client_ptr_;

  double server_wait_timeout_;

  // Services client
  ros::ServiceClient srv_transition_;
  ros::ServiceClient jps_service_client_;

  boost::mutex map_mutex_;
};