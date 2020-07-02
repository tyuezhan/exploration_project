#include <ddk_nav_2d/nav2d.h>
#include <ros/console.h>
#include <string>

using namespace ros;

#define PI 3.14159265

nav2d::nav2d(){
    
    NodeHandle nh;
    
    mapSubscriber = nh.subscribe("projected_map", 5, &nav2d::mapSubscriberCB, this);
    poseSubscriber = nh.subscribe("ddk/ground_truth/odom", 5000, &nav2d::poseSubscriberCB, this);
    octomapSubscriber = nh.subscribe("octomap_full", 5, &nav2d::octomapSubscriberCB, this);

    // Load planer
    nh.param("min_replanning_period", mMinReplanningPeriod, 5.0);
	nh.param("max_replanning_period", mMaxReplanningPeriod, 1.0);
    nh.param("exploration_strategy", mExplorationStrategy, std::string("NearestFrontierPlanner"));
    try{
		mPlanLoader = new PlanLoader("nav2d_navigator", "ExplorationPlanner");
		mExplorationPlanner = mPlanLoader->createInstance(mExplorationStrategy);
		ROS_INFO("Successfully loaded exploration strategy [%s].", mExplorationStrategy.c_str());
        
	}
	catch(pluginlib::PluginlibException& ex){
		ROS_ERROR("Failed to load exploration plugin! Error: %s", ex.what());
		mExploreActionServer = NULL;
		mPlanLoader = NULL;
	}

    // Explore action server
    nh.param("explore_action_topic", mExploreActionTopic, std::string(NAV_EXPLORE_ACTION));
    mExploreActionServer = new ExploreActionServer(mExploreActionTopic, boost::bind(&nav2d::receiveExploreGoal, this, _1), false);
    mExploreActionServer->start();
    
    // Get first map action server
	nh.param("getmap_action_topic", mGetMapActionTopic, std::string(NAV_GETMAP_ACTION));
    mGetFirstMapActionServer = new GetFirstMapActionServer(mGetMapActionTopic, boost::bind(&nav2d::receiveGetMapGoal, this, _1), false);
	mGetFirstMapActionServer->start();

    mStatus = NAV_ST_IDLE;
    lineTrackerStatus = NAV_ST_IDLE;
    line_tracker_min_jerk_client_= new ClientType(nh, "ddk/trackers_manager/line_tracker_min_jerk/LineTracker", true);
    float serverWaitTimeout;
    nh.param("Server_wait_timeout", serverWaitTimeout, 0.5f);
    if (!line_tracker_min_jerk_client_->waitForServer(ros::Duration(serverWaitTimeout))) {
    ROS_ERROR("LineTrackerMinJerk server not found.");
    }

    // Services
    line_tracker_min_jerk = "kr_trackers/LineTrackerMinJerk";
    active_tracker_ = "";
    srv_transition_ = nh.serviceClient<kr_tracker_msgs::Transition>("ddk/trackers_manager/transition");

    // tf listener
    nh.param("map_frame", mMapFrame, std::string("ddk/odom"));
	nh.param("robot_frame", mRobotFrame, std::string("ddk/base_link"));
    mRobotFrame = mTfListener.resolve(mRobotFrame);
	mMapFrame = mTfListener.resolve(mMapFrame);

    // Planning related param
    nh.param("map_inflation_radius", mInflationRadius, 1.0);
    nh.param("robot_radius", mRobotRadius, 0.2);
    mCostObstacle = 100;
    // mCostLethal = (3.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;
    mCostLethal = 100;
    mapUpdated = false;
}

nav2d::~nav2d(){
    ROS_INFO("Bye");
    delete mExploreActionServer;
	delete mGetFirstMapActionServer;
	mExplorationPlanner.reset();
	delete mPlanLoader;
    delete line_tracker_min_jerk_client_;
}


void nav2d::mapSubscriberCB(const nav_msgs::OccupancyGrid &map){
    ROS_INFO("Map update received.");
    mCurrentMap.update(map);
    
    // if(mCellInflationRadius == 0){
		// ROS_INFO("Navigator is now initialized.");
		// mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();
		// mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();
	    // 	mInflationTool.computeCaches(mCellInflationRadius);
		// mCurrentMap.setLethalCost(mCostLethal);
	// }

    if (mapUpdated == false){
        ROS_INFO("Navigator is now initialized.");
        mCellRobotRadius = 10;
        mCurrentMap.setLethalCost(mCostLethal);
    }
    mapUpdated = true;

}

void nav2d::octomapSubscriberCB(const octomap_msgs::Octomap &octomap){
    ROS_INFO("Octomap update received.");
    mPlanner.updateOctomap(octomap);
}

void nav2d::poseSubscriberCB(const nav_msgs::Odometry::ConstPtr &odom){
    // ROS_INFO("pose update received.");
    // ROS_DEBUG("Pose update received.");
    // currentPose = odom;
    
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
    poseUpdated = true;

}


// TODO: receive explore goal. recive map goal

void nav2d::receiveGetMapGoal(const ddk_nav_2d::GetFirstMapGoal::ConstPtr &goal){
    ROS_INFO("nav2d: Received get first map goal.");
    Rate loopRate(2);

    if(mStatus != NAV_ST_IDLE){
		ROS_WARN("Navigator is busy!");
		mGetFirstMapActionServer->setAborted();
		return;
	}

    if (mGetFirstMapActionServer->isPreemptRequested()){
        ROS_INFO("Get first map has been preempted externally.");
        mGetFirstMapActionServer->setPreempted();
        return;
    }

    ddk_nav_2d::GetFirstMapFeedback f;

    // Go straight
    float distance = 0;
    while(true){
        if (distance >= 2) break;
        mGetFirstMapActionServer->publishFeedback(f);
        if (lineTrackerStatus == NAV_ST_IDLE){
            goTo(1, 0, 0, 0, 0.0f, 0.0f, true);
            // goTo(2, 0, 0.5, 0, 0.0f, 0.0f, false);
            distance += 1;
        }
        spinOnce();
		loopRate.sleep();
    }
    

    // Turn 360, get first map
    float rotation = 0;
    while(true){
        if (rotation >= 6.282) break;
        // ROS_INFO("In the loop.");
        mGetFirstMapActionServer->publishFeedback(f);
        if (lineTrackerStatus == NAV_ST_IDLE){
            ROS_INFO("Move");
            goTo(0, 0, 0, 2.094, 0.0f, 0.0f, true);
            rotation += 2.094;
        }
        spinOnce();
		loopRate.sleep();
    }

    ROS_INFO("Set succeeded");
    mGetFirstMapActionServer->setSucceeded();
}

void nav2d::receiveExploreGoal(const ddk_nav_2d::ExploreGoal::ConstPtr &goal){
    ROS_INFO("nav2d: Received explore goal.");
    if(mStatus != NAV_ST_IDLE)
	{
		ROS_WARN("Navigator is busy!");
		mExploreActionServer->setAborted();
		return;
	}
    mStatus = NAV_ST_EXPLORING;
	unsigned int cycle = 0;
	unsigned int lastCheck = 0;
	unsigned int recheckCycles = mMinReplanningPeriod * mFrequency;
	unsigned int recheckThrottle = mMaxReplanningPeriod * mFrequency;

    // Move to exploration target
	// Rate loopRate(mFrequency);
    Rate loopRate(1);
	while(true){

        if (mExploreActionServer->isPreemptRequested()){
            ROS_INFO("Exploration has been preempted externally.");
			mExploreActionServer->setPreempted();
            return;
        }

        // get current map index
        if (!getMapIndex()){
            ROS_ERROR("Exploration failed, could not get current position.");
			mExploreActionServer->setAborted();
			return;
        }

        // recheck for exploration target
        bool reCheck = lastCheck == 0 || (recheckCycles && (cycle - lastCheck > recheckCycles));
        bool goalReached = false;
        if (lineTrackerStatus == NAV_ST_IDLE){
            goalReached = true;
            ROS_INFO("Set goal reached true. can continue explore");
        } 

        if (reCheck || goalReached){
            WallTime startTime = WallTime::now();
            lastCheck = cycle;

            // bool success = false;
            if(preparePlan()){
                // ROS_INFO("Enter prepare plan true");
                GridMap currentMapCopy = mCurrentMap;
                int result = mExplorationPlanner->findExplorationTarget(&currentMapCopy, mStartPoint, mGoalPoint);
                ROS_INFO("result: %d", result);
                switch(result){
                    case EXPL_TARGET_SET:
                        ROS_INFO("EXPL TARGET SET");
                        if (lineTrackerStatus == NAV_ST_IDLE){
                            unsigned int goal_x = 0, goal_y = 0;
                            if(mCurrentMap.getCoordinates(goal_x,goal_y,mGoalPoint)){
                                float worldGoalX = mCurrentMap.getOriginX() + (((double)goal_x+0.5) * mCurrentMap.getResolution());
                                float worldGoalY = mCurrentMap.getOriginY() + (((double)goal_y+0.5) * mCurrentMap.getResolution());
                                float worldGoalZ = 0.4;
                                float yaw;
                                // unsigned int current_x = (pos_(0) - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
	                            // unsigned int current_y = (pos_(1) - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
                                // yaw = headTowards(pos_(0), pos_(1), mGoalPoint);

                                yaw = atan2(pos_(1)-worldGoalY, pos_(0)-worldGoalX);
                                yaw += PI;
                                if(yaw < -PI) yaw += 2*PI;
                                if(yaw > PI) yaw -= 2*PI;
                                
                                lineTrackerStatus = NAV_ST_TURNING;
                                while(true){
                                    if (lineTrackerStatus == NAV_ST_IDLE) break;
                                    ROS_INFO("turning");
                                    if (lineTrackerStatus == NAV_ST_TURNING){
                                        goTo(pos_(0), pos_(1), pos_(2), yaw, 0.0f, 0.0f, false);
                                    }                        
                                    spinOnce();
                                    loopRate.sleep();
                                }
                                // mStatus = NAV_ST_EXPLORING;
                                goTo(worldGoalX, worldGoalY, worldGoalZ, yaw, 0.0f, 0.0f, false);
                                ROS_INFO("goal: %f, %f, %f, %f", worldGoalX, worldGoalY, worldGoalZ, yaw);
                                mStatus = NAV_ST_EXPLORING;
                            }
                        }
                        break;
                    case EXPL_FINISHED:
                        {
                            ddk_nav_2d::ExploreResult r;
                            r.final_pose.x = pos_(0);
                            r.final_pose.y = pos_(1);
                            r.final_pose.theta = yaw_;
                            mExploreActionServer->setSucceeded(r);
                        }
                        ROS_INFO("Exploration has finished.");
                        return;
                    case EXPL_WAITING:
                        mStatus = NAV_ST_WAITING;
                        ROS_INFO("Exploration is waiting.");
					    break;
                    case EXPL_FAILED:
                        break;
                    default:
					ROS_ERROR("Exploration planner returned invalid status code: %d!", result);
                }
            }

            if(mStatus == NAV_ST_EXPLORING)
			{
				if(lineTrackerStatus == NAV_ST_NAVIGATING)
				{
                    ROS_INFO("moving.");
					WallTime endTime = WallTime::now();
					// WallDuration d = endTime - startTime;
					// ROS_DEBUG("Exploration planning took %.09f seconds", d.toSec());
				}else
				{
					mExploreActionServer->setAborted();
					ROS_WARN("Exploration has failed!");
                    mStatus = NAV_ST_IDLE;
					return;
				}
			}
        }

        if(mStatus == NAV_ST_EXPLORING)
		{
			// Publish feedback via ActionServer
			if(cycle%10 == 0)
			{
				ddk_nav_2d::ExploreFeedback fb;
				fb.robot_pose.x = pos_(0);
				fb.robot_pose.y = pos_(1);
				fb.robot_pose.theta = yaw_;
				mExploreActionServer->publishFeedback(fb);
			}

		}

		// Sleep remaining time
		cycle++;
		spinOnce();
		loopRate.sleep();
		// if(loopRate.cycleTime() > ros::Duration(1.0 / mFrequency))
			// ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",mFrequency, loopRate.cycleTime().toSec());
    }
}


bool nav2d::goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative){
    kr_tracker_msgs::LineTrackerGoal goal;
    goal.x = x;
    goal.y = y;

    goal.relative = relative;
    //Convert relative translation in body frame to global frame
    if(relative)
    {
        goal.x = x * std::cos(yaw_) - y * std::sin(yaw_);
        goal.y = x * std::sin(yaw_) + y * std::cos(yaw_);
    }

    goal.z = z;
    goal.yaw = yaw;
    goal.v_des = v_des;
    goal.a_des = a_des;

    line_tracker_min_jerk_client_->sendGoal(goal, boost::bind(&nav2d::tracker_done_callback, this, _1, _2), ClientType::SimpleActiveCallback(), ClientType::SimpleFeedbackCallback());
    lineTrackerStatus = NAV_ST_NAVIGATING;

    return transition(line_tracker_min_jerk);
}


void nav2d::tracker_done_callback(const actionlib::SimpleClientGoalState& state, const kr_tracker_msgs::LineTrackerResultConstPtr& result){
    ROS_INFO("Goal reached.");
    lineTrackerStatus = NAV_ST_IDLE;
};


bool nav2d::transition(const std::string &tracker_str) {
  kr_tracker_msgs::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd) && transition_cmd.response.success) {
    active_tracker_ = tracker_str;
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }

  return false;
}


bool nav2d::getMapIndex(){
    tf::StampedTransform transform;
	try
	{
		mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
	}catch(tf::TransformException ex)
	{
		ROS_ERROR("Could not get robot position: %s", ex.what());
		return false;
	}
    double world_x = transform.getOrigin().x();
	double world_y = transform.getOrigin().y();
	double world_theta = getYaw(transform.getRotation());

	unsigned int current_x = (world_x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
	unsigned int current_y = (world_y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
	unsigned int i;

    mCurrentMap.getIndex(current_x, current_y, i);
    signed char valueHere = mCurrentMap.getData(current_x, current_y);
    // ROS_INFO("Check current position map value: %u", valueHere);
    mStartPoint = i;
    // Can delete these
    // mCurrentDirection = world_theta;
	// mCurrentPositionX = world_x;
	// mCurrentPositionY = world_y;
    ROS_INFO("Get map index: %f, %f, %d, %d, %d", world_x, world_y, current_x, current_y, i);
    return true;
}

bool nav2d::preparePlan(){
	// Where am I?
	if(!getMapIndex()) return false;
	
	// Clear robot footprint in map
	unsigned int x = 0, y = 0;
	if(mCurrentMap.getCoordinates(x, y, mStartPoint))
		for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
			for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
				mCurrentMap.setData(x+i, y+j, 0);
	ROS_INFO("prepare plan, set Startpoint as %d", mStartPoint);
	// mInflationTool.inflateMap(&mCurrentMap);
	return true;
}


// float nav2d::headTowards(unsigned int current_x, unsigned int current_y, unsigned int target){
//     unsigned int x = 0, y = 0;
// 	if(!mCurrentMap.getCoordinates(x, y, target))
// 	{
// 		ROS_ERROR("Turn head towards goalpoint failed.");
// 		return 0;
// 	}
// 	double map_angle = atan2((double)y - current_y, (double)x - current_x);
	
// 	double angle = map_angle - yaw_;
// 	if(angle < -PI) angle += 2*PI;
// 	if(angle > PI) angle -= 2*PI;
//     return angle;
// }