#include <ros/ros.h>

#include <ddk_nav_2d/nav2d.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Navigator");
	ros::NodeHandle n;
	
	// RobotNavigator robNav;
	nav2d nav2D;

	ros::spin();
	return 0;
}
