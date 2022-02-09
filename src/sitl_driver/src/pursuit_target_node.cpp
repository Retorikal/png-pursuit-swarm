#include <ros/ros.h>
#include <sitl_driver/Pilot.hpp>

int main(int argc, char **argv){
	ros::init(argc, argv, "chase_target", 0);
  
  ros::Rate rate(40);
  ros::NodeHandle nh;
	ros::AsyncSpinner spinner(4);
  ros::Publisher pos_pub = nh.advertise<geometry_msgs::Pose>("/pursuit_target", 500);

	spinner.start();

  while (true){
    rate.sleep();
  }

	return 0;
}