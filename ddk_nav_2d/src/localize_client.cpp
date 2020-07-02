#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ddk_nav_2d/LocalizeAction.h>
#include <std_srvs/Trigger.h>

#include <ddk_nav_2d/commands.h>

typedef actionlib::SimpleActionClient<ddk_nav_2d::LocalizeAction> LocalizeClient;

LocalizeClient* gLocalizeClient;

bool receiveCommand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	ddk_nav_2d::LocalizeGoal goal;
	goal.velocity = 0.5;
	gLocalizeClient->sendGoal(goal);
	res.success = true;
	res.message = "Send LocalizeGoal to Navigator.";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Localize");
	ros::NodeHandle n;
	
	ros::ServiceServer cmdServer = n.advertiseService(NAV_LOCALIZE_SERVICE, &receiveCommand);
	gLocalizeClient = new LocalizeClient(NAV_LOCALIZE_ACTION, true);
	gLocalizeClient->waitForServer();
	
	ros::spin();
	
	delete gLocalizeClient;
	return 0;
}
