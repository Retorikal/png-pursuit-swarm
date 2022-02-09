#include <ros/ros.h>
#include <cmath>
#include <mavros_msgs/StreamRate.h>
#include <sitl_driver/Pilot.hpp>
#include <sitl_driver/missions.hpp>

namespace sitl_driver{
	Pilot::Pilot(){
		mavros_pose_sub = nh.subscribe("/mavros/local_position/pose", 500, &Pilot::statePoseCb, this);
		mavros_vel_sub = nh.subscribe("/mavros/local_position/velocity_local", 500, &Pilot::stateVelCb, this);
		mavros_setvel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 500);
		state_pose_pub = nh.advertise<geometry_msgs::Pose>("/debug/pose", 500);
		state_vel_pub = nh.advertise<geometry_msgs::Twist>("/debug/vel", 500);

		ros::ServiceClient streamRateClient =	nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
		mavros_msgs::StreamRate streamRateParam;
		streamRateParam.request.message_rate = mavros_rate;

		if(streamRateClient.call(streamRateParam)){
			ROS_INFO("mavros stream rate updated to %d", streamRateParam.request.message_rate);
		}
		else{
			ROS_ERROR("Failed to set mavros stream rate!");
		}
	}

	Pilot::~Pilot(){}

	void Pilot::execute(){
		ROS_INFO("----Misssion start!----");
		
		missions::takeoff();
		//missions::png_chase(this);

		ROS_INFO("----Misssion end!----");
	}

	void Pilot::set_targetvel(tf2::Vector3 targetvel){;
		geometry_msgs::TwistStamped targettwist_msg;
		targettwist_msg.twist.linear = tf2::toMsg(targetvel);
		
		mavros_setvel_pub.publish<geometry_msgs::TwistStamped>(targettwist_msg);
	}

	void Pilot::statePoseCb(const geometry_msgs::PoseStampedConstPtr& msg){
		geometry_msgs::PoseStamped pose_stamp = *(msg.get());
		tf2::fromMsg(pose_stamp.pose.orientation, orientation);
		tf2::fromMsg(pose_stamp.pose.position, position);  
		state_pose_pub.publish<geometry_msgs::Pose>(pose_stamp.pose);
	}

	void Pilot::stateVelCb(const geometry_msgs::TwistStampedConstPtr& msg){
		geometry_msgs::TwistStamped tmp_twist = *(msg.get());
		tf2::fromMsg(tmp_twist.twist.linear, v_lin);
		tf2::fromMsg(tmp_twist.twist.angular, v_rot);
		state_vel_pub.publish<geometry_msgs::Twist>(tmp_twist.twist);
	}

} // namespace sitl_drive

