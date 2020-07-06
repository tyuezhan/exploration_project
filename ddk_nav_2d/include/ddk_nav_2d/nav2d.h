#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <ddk_nav_2d/GridMap.h>
#include <ddk_nav_2d/commands.h>
#include <ddk_nav_2d/GetFirstMapAction.h>
#include <ddk_nav_2d/MapInflationTool.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>

// include action
#include <ddk_nav_2d/ExploreAction.h>
#include <ddk_nav_2d/GetFirstMapAction.h>

// TODO: Delete unnecessary part in Exploration planner
#include <ddk_nav_2d/ExplorationPlanner.h>

#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/Transition.h>

#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <ddk_nav_2d/ddkPlanner.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <kr_replanning_msgs/TrackPathAction.h>


typedef actionlib::SimpleActionServer<ddk_nav_2d::GetFirstMapAction> GetFirstMapActionServer;
typedef actionlib::SimpleActionServer<ddk_nav_2d::ExploreAction> ExploreActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;


const int POW = 6;


class nav2d{

public:

    nav2d();
    ~nav2d();

    typedef Eigen::Vector3f    Vec3;
    typedef Eigen::Quaternionf Quat;
    Vec3 pos() { return pos_; }
    Vec3 vel() { return vel_; }

    float yaw() { return yaw_; } 

    ros::Subscriber mapSubscriber;
    ros::Subscriber poseSubscriber;

    void mapSubscriberCB(const nav_msgs::OccupancyGrid &map);
    void poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom);
    void receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal);
	void receiveGetMapGoal(const ddk_nav_2d::GetFirstMapGoal::ConstPtr &goal);
    
    bool goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative);
    bool transition(const std::string &tracker_str);
    bool getMapIndex();
    bool preparePlan();

    //replanning
    bool getJpsTraj(const double& traj_time, const Eigen::Affine3f& o_w_transform, geometry_msgs::PoseStamped& min_cost_pt);
    // bool getJpsTraj(const double& traj_time, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped& goal);
    void updateTrackingPath(const Eigen::Vector4d& limits, const geometry_msgs::Point& start, ewok::PolynomialTrajectory3D<10>::Ptr& traj);

    
    //3D
    ddkPlanner mPlanner;

private:

    int mStatus;
    int lineTrackerStatus;
    Vec3 pos_, vel_;
    float yaw_, yaw_dot_;
    Quat odom_q_, imu_q_;
    ros::Time last_odom_t_;

    // Map related things
    MapInflationTool mInflationTool;

    bool mapUpdated;
    bool poseUpdated;
    GridMap mCurrentMap;
    nav_msgs::Odometry currentPose;

    tf::TransformListener mTfListener;
    std::string mMapFrame;
	std::string mRobotFrame;

    // Planning related param
	double mFrequency;
	double mInflationRadius;
	double mRobotRadius;
	unsigned int mCellInflationRadius;
	unsigned int mCellRobotRadius;

    signed char mCostObstacle;
	signed char mCostLethal;

    // action Server
    GetFirstMapActionServer* mGetFirstMapActionServer;
    ExploreActionServer* mExploreActionServer;
    
    std::string mExploreActionTopic;
	std::string mGetMapActionTopic;

    // action Client 
    typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> ClientType;
    void tracker_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::LineTrackerResultConstPtr& result);
    ClientType* line_tracker_min_jerk_client_;

    // Services client
    ros::ServiceClient srv_transition_;
    std::string active_tracker_;
    std::string line_tracker_min_jerk;
    
    // Plan
    std::string mExplorationStrategy;
	boost::shared_ptr<ExplorationPlanner> mExplorationPlanner;
    PlanLoader* mPlanLoader;
    double mMinReplanningPeriod;
	double mMaxReplanningPeriod;
    unsigned int mGoalPoint;
	unsigned int mStartPoint;

    // delete these three later?
    double mCurrentDirection;
	double mCurrentPositionX;
	double mCurrentPositionY;

    // replanning
    ewok::PolynomialTrajectory3D<10>::Ptr local_traj_;
    ewok::PolynomialTrajectory3D<10>::Ptr orig_global_traj_;
    ewok::PolynomialTrajectory3D<10>::Ptr global_traj_;
    ewok::UniformBSpline3DOptimization<6>::Ptr spline_optimization_;
    ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb_;
    ros::ServiceClient jps_service_client_;
    double lookahead_time_;
    double max_velocity_, max_acceleration_;
    ros::Publisher global_traj_marker_pub_;
    double initial_traj_yaw_;
    double dt_;
    int num_opt_points_;
    double edrb_size_;
    double resolution_;
    double distance_threshold_;

    typedef actionlib::SimpleActionClient<kr_replanning_msgs::TrackPathAction> trackPathClientType;
    trackPathClientType* track_path_action_client_;
    void track_path_done_callback(const actionlib::SimpleClientGoalState& state, const kr_replanning_msgs::TrackPathResultConstPtr& result);
    int trackPathStatus;
    bool trackPath(nav_msgs::Path plannedPath);
};