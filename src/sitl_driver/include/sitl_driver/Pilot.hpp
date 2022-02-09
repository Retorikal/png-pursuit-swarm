#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#pragma once

namespace sitl_driver{
	class Pilot{
	  ros::NodeHandle nh;
	  ros::Subscriber mavros_pose_sub;
	  ros::Subscriber mavros_vel_sub;
		ros::Publisher mavros_setvel_pub;
		ros::Publisher state_pose_pub;
		ros::Publisher state_vel_pub;

		int mavros_rate = 60;
		
		void statePoseCb(const geometry_msgs::PoseStampedConstPtr& msg);
		void stateVelCb(const geometry_msgs::TwistStampedConstPtr& msg);

	public:
    tf2::Quaternion orientation;
    tf2::Vector3 position;
    tf2::Vector3 v_lin;
    tf2::Vector3 v_rot;
		ros::Time timestamp;
		Pilot();
		~Pilot();
		void execute();
		void set_targetvel(tf2::Vector3 targetvel);
	};
}