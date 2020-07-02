#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <ddk_nav_2d/GridMap.h>
#include <ddk_nav_2d/commands.h>
#include <ddk_nav_2d/GetFirstMapAction.h>
#include <ddk_nav_2d/MapInflationTool.h>

#include <Eigen/Geometry>

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


typedef actionlib::SimpleActionServer<ddk_nav_2d::GetFirstMapAction> GetFirstMapActionServer;
typedef actionlib::SimpleActionServer<ddk_nav_2d::ExploreAction> ExploreActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;



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
    // ros::Subscriber octomapSubscriber;

    void mapSubscriberCB(const nav_msgs::OccupancyGrid &map);
    // void octomapSubscriberCB(const octomap_msgs::Octomap &octomap);
    void poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom);
    void receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal);
	void receiveGetMapGoal(const ddk_nav_2d::GetFirstMapGoal::ConstPtr &goal);
    
    bool goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative);
    bool transition(const std::string &tracker_str);
    bool getMapIndex();
    bool preparePlan();
    // float headTowards(unsigned int current_x, unsigned int current_y, unsigned int target);

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
    // ros::NodeHandle nh1;
    // ClientType line_tracker_min_jerk_client_(nh1, "ddk/trackers_manager/line_tracker_min_jerk/LineTracker", true);;
    // line_tracker_min_jerk_client_
    ClientType* line_tracker_min_jerk_client_;

    // Services
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

};