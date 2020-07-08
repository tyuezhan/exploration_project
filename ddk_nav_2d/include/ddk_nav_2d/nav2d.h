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

#include <ddk_nav_2d/GridMap.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

// include action
#include <ddk_nav_2d/ExploreAction.h>

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

#define NAV_STOP_SERVICE "Stop"
#define NAV_PAUSE_SERVICE "Pause"
#define NAV_EXPLORE_SERVICE "StartExploration"
#define NAV_GETMAP_SERVICE  "StartMapping"
#define NAV_LOCALIZE_SERVICE  "StartLocalization"
#define NAV_GOAL_TOPIC      "goal"
#define NAV_STATUS_TOPIC    "nav_status"
#define NAV_MOVE_ACTION     "MoveTo"
#define NAV_EXPLORE_ACTION  "Explore"
#define NAV_GETMAP_ACTION   "GetFirstMap"
#define NAV_LOCALIZE_ACTION "Localize"

#define NAV_ST_IDLE	      0
#define NAV_ST_NAVIGATING 1
#define NAV_ST_EXPLORING  4
#define NAV_ST_WAITING    5
#define NAV_ST_RECOVERING 6
#define NAV_ST_TURNING    7

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
  typedef actionlib::SimpleActionServer<ddk_nav_2d::ExploreAction> ExploreServerType;
  typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> LineClientType;
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::TrajectoryTrackerAction> TrajectoryClientType;
  typedef actionlib::SimpleActionClient<kr_replanning_msgs::TrackPathAction> TrackPathClientType;

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

  // 2D map related param
  bool map_updated_;
  bool pose_updated_;
  GridMap current_map_;

  // TF param
  tf::TransformListener tf_listener_;
  std::string map_frame_;
  std::string robot_frame_;

  // 2D frontier exploration related param
  std::string exploration_strategy_;
  boost::shared_ptr<ExplorationPlanner> exploration_planner_;
  std::unique_ptr<PlanLoader> plan_loader_ptr_;

  double min_replanning_period_;
  double max_replanning_period_;
  unsigned int goal_point_;
  unsigned int start_point_;
  double frequency_;
  double robot_radius_;
  unsigned int cell_robot_radius_;
  signed char cost_lethal_;


  // action Server and param:  exploration server
  std::unique_ptr<ExploreServerType> explore_action_server_ptr_;

  std::string explore_action_topic_;
  std::string get_map_action_topic_;

  // action Client: line tracker; trajectory tracker 
  std::unique_ptr<LineClientType> line_tracker_min_jerk_client_ptr_;
  std::unique_ptr<TrajectoryClientType> traj_tracker_client_ptr_;
  std::string line_tracker_min_jerk_;
  std::string traj_tracker_;

  // Services client and param: tracker transition; jps planner
  ros::ServiceClient srv_transition_;
  std::string active_tracker_;

  ros::ServiceClient jps_service_client_;
  std::unique_ptr<TrackPathClientType> track_path_action_client_ptr_;
};