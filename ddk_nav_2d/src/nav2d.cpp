#include <ddk_nav_2d/nav2d.h>
#include <ros/console.h>
#include <string>

using namespace ros;

#define PI 3.14159265

nav2d::nav2d(){
    
    NodeHandle nh;
    
    mapSubscriber = nh.subscribe("projected_map", 5, &nav2d::mapSubscriberCB, this);
    poseSubscriber = nh.subscribe("ddk/ground_truth/odom", 5000, &nav2d::poseSubscriberCB, this);

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

    // replanning
    // jps_service_client_ = nh.serviceClient<nav_msgs::GetPlan>("jps_plan_service");
    // nh.param("lookahead_time", lookahead_time_, 3.0);
    // nh.param("max_velocity", max_velocity_, 0.5);
    // nh.param("max_acceleration", max_acceleration_, 0.6);

    // Eigen::Vector4d limits(max_velocity_, max_acceleration_, 0, 0);

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

                                yaw = atan2(pos_(1)-worldGoalY, pos_(0)-worldGoalX);
                                yaw += PI;
                                if(yaw < -PI) yaw += 2*PI;
                                if(yaw > PI) yaw -= 2*PI;
                                
                                // lineTrackerStatus = NAV_ST_TURNING;
                                // while(true){
                                //     if (lineTrackerStatus == NAV_ST_IDLE) break;
                                //     ROS_INFO("turning");
                                //     if (lineTrackerStatus == NAV_ST_TURNING){
                                //         goTo(pos_(0), pos_(1), pos_(2), yaw, 0.0f, 0.0f, false);
                                //     }                        
                                //     spinOnce();
                                //     loopRate.sleep();
                                // }
                                // // mStatus = NAV_ST_EXPLORING;
                                // goTo(worldGoalX, worldGoalY, worldGoalZ, yaw, 0.0f, 0.0f, false);
                                // ROS_INFO("goal: %f, %f, %f, %f", worldGoalX, worldGoalY, worldGoalZ, yaw);
                                // mStatus = NAV_ST_EXPLORING;

                                // get current odom transform
                                tf::StampedTransform transform;
                                try{
                                    mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
                                }
                                catch (tf::TransformException &ex){
                                    ROS_INFO("Couldn't get current odom transform");
                                    ROS_WARN("%s",ex.what());
                                    return;
                                }

                                // Eigen::Affine3d dT_o_w;
                                // tf::transformTFToEigen(transform, dT_o_w);
                                // Eigen::Affine3f Tf_odom_to_world = dT_o_w.cast<float>();
                                double roll = 0.0;
                                double pitch = 0.0;
                                tf::Quaternion goalQuaternion = createQuaternionFromRPY(roll, pitch, yaw);
                                geometry_msgs::PoseStamped goal;
                                goal.header.frame_id = mMapFrame;
                                goal.header.stamp = ros::Time::now();
                                goal.pose.position.x = worldGoalX;
                                goal.pose.position.y = worldGoalY;
                                goal.pose.position.z = worldGoalZ;
                                goal.pose.orientation = goalQuaternion;
                                double local_time_ = 0.0;

                                geometry_msgs::PoseStamped start;
                                tf::Quaternion startQuaternion = createQuaternionFromRPY(roll, pitch, yaw_);
                                start.header.frame_id = mMapFrame;
                                start.header.stamp = ros::Time::now();
                                start.pose.position.x = pos_(0);
                                start.pose.position.y = pos_(1);
                                start.pose.position.z = pos_(2);
                                start.pose.orientation = startQuaternion;
                                // bool ret = getJpsTraj(local_time_, Tf_odom_to_world, min_cost_pt);


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


// bool nav2d::getJpsTraj(const double& traj_time, const Eigen::Affine3f& o_w_transform, geometry_msgs::PoseStamped& min_cost_pt){
bool nav2d::getJpsTraj(const double& traj_time, geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped& goal){
    nav_msgs::GetPlan jps_srv;
    jps_srv.request.start = start;
    // jps_srv.request.start = min_cost_pt;
    // jps_srv.request.start.pose.position.x = o_w_transform.translation()[0];
    // jps_srv.request.start.pose.position.y = o_w_transform.translation()[1];
    // jps_srv.request.start.pose.position.z = o_w_transform.translation()[2];
    jps_srv.request.goal = goal;
    if (jps_service_client_.call(jps_srv))
    {
    ROS_INFO("Successful jps service call");

    /*for(int i = 0; i < jps_srv.response.plan.poses.size(); i++)
    {
        geometry_msgs::PoseStamped ps = jps_srv.response.plan.poses[i];
        path.push_back(Eigen::Vector3d(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
    }*/

    if(jps_srv.response.plan.poses.size()==0)
    {
        ROS_ERROR("JPS did not return a path");
        return false;
    }

    double lookah_join_time = traj_time + lookahead_time_*2.5;
    Eigen::Vector3d lookah_join = orig_global_traj_->evaluate(lookah_join_time);
    double traj_lt, lookah_join_lt;
    size_t traj_seg_num, lookah_join_seg_num;
    orig_global_traj_->findSegmentNumAndLocalTime(traj_time - 2.0, traj_lt, traj_seg_num);
    orig_global_traj_->findSegmentNumAndLocalTime(lookah_join_time, lookah_join_lt, lookah_join_seg_num);

    //ROS_WARN("l seg %zu tim %g join seg %zu tim %g", lookah_seg_num, lookah_lt, lookah_join_seg_num, lookah_join_lt);

    visualization_msgs::MarkerArray local_traj;
    local_traj.markers.resize(orig_global_traj_->numSegments()+1);
    visualization_msgs::Marker &traj_marker = local_traj.markers[0];
    traj_marker.header.frame_id = "world";
    traj_marker.ns = "gt";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::POINTS;
    traj_marker.action = visualization_msgs::Marker::MODIFY;
    traj_marker.scale.x = 0.01;
    traj_marker.scale.y = 0.01;
    traj_marker.scale.z = 0.01;
    traj_marker.color.a = 1.0;
    traj_marker.lifetime = ros::Duration(0);
    traj_marker.color.r = 1;
    traj_marker.color.g = 0;
    traj_marker.color.b = 1;

    //local_traj.markers[0].points.reserve(jps_srv.response.plan.poses.size());
    for(int i = 0; i < jps_srv.response.plan.poses.size(); i++)
    {
        geometry_msgs::PoseStamped ps = jps_srv.response.plan.poses[i];
        //path.push_back(Eigen::Vector3d(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z));
        local_traj.markers[0].points.push_back(ps.pose.position);
    }

    //Add connecting path from jps to lookah_join point
    geometry_msgs::Point jps_goal = local_traj.markers[0].points.back();
    geometry_msgs::Point jps_start = local_traj.markers[0].points.front();

    std::vector<geometry_msgs::Point>::iterator it = local_traj.markers[0].points.begin();
    if(local_traj.markers[0].points.size() > 2){
        it += 1;
        jps_start = *it; //Take a point slightly front to avoid going back
    }

    float diff_x = lookah_join[0] - jps_goal.x;
    float diff_y = lookah_join[1] - jps_goal.y;
    float diff_z = lookah_join[2] - jps_goal.z;
    int num_steps = int(sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z)/0.2);
    geometry_msgs::Point interm_pt; //TODO instead use vector and direction
    for (int i = 1; i < num_steps; i++)
    {
        interm_pt.x = diff_x*i/num_steps + jps_goal.x;
        interm_pt.y = diff_y*i/num_steps + jps_goal.y;
        interm_pt.z = diff_z*i/num_steps + jps_goal.z;
        local_traj.markers[0].points.push_back(interm_pt);
    }

    float dt = 0.3;
    ROS_WARN("Get removed seg traj %d %d", local_traj.markers.size(), orig_global_traj_->numSegments());
    orig_global_traj_->getRemovedSegmentTraj(local_traj, traj_lt, traj_seg_num, lookah_join_lt, lookah_join_seg_num, "gt", Eigen::Vector3d(1,0,1), dt);

    //global_traj_marker_pub_.publish(local_traj);

    Eigen::Vector4d limits(max_velocity_, 0, 0, 0);
    ewok::Polynomial3DOptimization<10> to(limits*0.6);
    ewok::Polynomial3DOptimization<10>::Vector3Array path;
    for (int i=1; i<local_traj.markers.size();i++)
    {
        for (int j=0; j<local_traj.markers[i].points.size();j++)
        {
        geometry_msgs::Point pt = local_traj.markers[i].points[j];
        path.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }
    }

    local_traj_ = to.computeTrajectory(path);

    /*
    visualization_msgs::MarkerArray local_marker;
    local_traj_->getVisualizationMarkerArray(local_marker, "gt", Eigen::Vector3d(1,0,1));
    global_traj_marker_pub_.publish(local_marker);
    */

    updateTrackingPath(limits, jps_start, local_traj_);
    return true;
    }
    else
    {
    ROS_ERROR("Failed to jps call service"); //TODO handle service not present, planning errors case properly
    return false;
    }
}


void nav2d::updateTrackingPath(const Eigen::Vector4d& limits, const geometry_msgs::Point& start, ewok::PolynomialTrajectory3D<10>::Ptr& traj)
{
  global_traj_ = traj;

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
  global_traj_marker_pub_.publish(traj_marker);

  //Get initial traj yaw
  std::vector<geometry_msgs::Point> traj_pts = traj_marker.markers[0].points;

  initial_traj_yaw_ = 0.0;
  if(traj_pts.size()>2)
  {
    geometry_msgs::Point pt1 = traj_pts[0];
    geometry_msgs::Point pt2 = traj_pts[1];
    initial_traj_yaw_ = std::atan2(pt2.y - pt1.y, pt2.x - pt1.x);
  }
  ROS_WARN("Initial yaw %g", angles::to_degrees(initial_traj_yaw_));

  Eigen::Vector3d st(start.x, start.y, start.z);
  spline_optimization_.reset(new ewok::UniformBSpline3DOptimization<6>(st, traj, dt_));
  float dt = 0.0;
  for (int i = 0; i < num_opt_points_; i++)
  {
    Eigen::Vector3d lookah = traj->evaluate(dt);
    spline_optimization_->addControlPoint(lookah);
    //spline_optimization_->addControlPoint(st);
    dt += 0.05;
  }

  spline_optimization_->setNumControlPointsOptimized(num_opt_points_);
  spline_optimization_->setDistanceBuffer(edrb_);
  spline_optimization_->setDistanceThreshold(distance_threshold_);
  spline_optimization_->setLimits(limits);

  path_initialized_ = true;
  path_tracking_ = true;
  traj_reset_time_ = 0.0;

  ROS_INFO("Received new waypoints, num segments");// %d", traj_marker.markers.size());
}