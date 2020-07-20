#include <ros/ros.h>

#include <ddk_nav_2d/nav2d.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigator");
	ros::NodeHandle n;
	
	// RobotNavigator robNav;
	Nav2D Nav2D;

	ros::spin();
	return 0;
}
