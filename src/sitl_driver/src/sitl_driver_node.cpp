
#include <ros/ros.h>
#include <sitl_driver/Pilot.hpp>

int main(int argc, char **argv){
	ros::init(argc, argv, "sitl_pilot", 0);
	ros::AsyncSpinner spinner(4);

	sitl_driver::Pilot pilot;
	pilot.execute();
	
	spinner.start();
	ros::waitForShutdown();
	return 0;
}