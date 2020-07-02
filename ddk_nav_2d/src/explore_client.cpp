#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ddk_nav_2d/ExploreAction.h>
#include <std_srvs/Trigger.h>

#include <ddk_nav_2d/commands.h>

typedef actionlib::SimpleActionClient<ddk_nav_2d::ExploreAction> ExploreClient;

ExploreClient* gExploreClient;

bool receiveCommand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ddk_nav_2d::ExploreGoal goal;
	gExploreClient->sendGoal(goal);
	res.success = true;
	res.message = "Send ExploreGoal to Navigator.";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Explore");
	ros::NodeHandle n;
	
	ros::ServiceServer cmdServer = n.advertiseService(NAV_EXPLORE_SERVICE, &receiveCommand);
	gExploreClient = new ExploreClient(NAV_EXPLORE_ACTION, true);
	gExploreClient->waitForServer();
	
	ros::spin();
	
	delete gExploreClient;
	return 0;
}
