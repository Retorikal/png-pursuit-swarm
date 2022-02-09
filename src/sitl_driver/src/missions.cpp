#include <sitl_driver/Pilot.hpp>
#include <sitl_driver/missions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/CommandTOL.h>

namespace missions{
  int png_chase(sitl_driver::Pilot &pilot){
    ros::NodeHandle nh;
    ros::Rate loop_rate(4);
    std::string target = "/target/local_position/pose";
    double pursuit_velocity = 5;
    double bearing_angle_k = 4.5;

    // Target properties placeholder
    tf2::Quaternion target_orientation;
    tf2::Vector3 target_position;
    std::vector<ros::Time> target_times(2);
    std::vector<tf2::Vector3> deltas(2);
    unsigned long long int target_data_count;
    double inv_dt;

    // Target properties callback
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(target, 500, 
      [&](const geometry_msgs::PoseStampedConstPtr& msg){ 
        geometry_msgs::PoseStamped pose_stamped = *(msg.get());
        tf2::fromMsg(pose_stamped.pose.orientation, target_orientation);
        tf2::fromMsg(pose_stamped.pose.position, target_position);  

        // Update target t and t-1 properties.
        deltas = {deltas[1], pilot.position - target_position};
        target_times = {target_times[1], pose_stamped.header.stamp};
        inv_dt = 1/(target_times[1] - target_times[0]).sec;

        target_data_count ++;
      }
    );

    while(true){
      // Hold execution until new target data is acquired
      static ros::Time current_time;
      if(current_time == target_times[1])
        continue;
      else
        current_time = target_times[1];

      // Allow at least 2 steps of information to be acquired before proceeding
      if(target_data_count <= 1){
        loop_rate.sleep();
        continue;
      }

      // Start calculation
      tf2::Vector3 gamma = pilot.v_lin;

      // Calculate target heading based on 3-dimensional Proportional Navigation Law
      double sec_u = 1/std::cos(gamma.angle(deltas[1]));
      tf2::Vector3 unit_v = tf2::tf2Cross(gamma, deltas[1]).normalized();
      tf2::Vector3 unit_u_gamma = tf2::tf2Cross(unit_v, gamma).normalized();

      // Only God knows what I write here setelah aku keluar dari CMD. This is for projecting ddelta to u and v plane.
      // Ref: https://math.stackexchange.com/questions/2207665/projecting-an-angle-from-one-plane-to-another-plane
      tf2::Vector3 ddelta_normal = tf2::tf2Cross(deltas[0], deltas[1]);
      double tan_ddelta = tan((double)deltas[1].angle(deltas[0]) * inv_dt);
      double normald_ddelta_u = ddelta_normal.angle(unit_v); // unit_v is plane u's normal vector
      double normald_ddelta_v = ddelta_normal.angle(gamma); // unit_gamma is plane v's normal vector
      double ddelta_u = atan(tan_ddelta/cos(normald_ddelta_u));
      double ddelta_v = atan(tan_ddelta/cos(normald_ddelta_v));

      // With factors done.. final-calculate PNG (Adler, 1956)
      tf2::Vector3 factor_u = ddelta_u * sec_u * unit_u_gamma;
      tf2::Vector3 factor_v = ddelta_v * unit_v;
      tf2::Vector3 dunit_gamma_target = bearing_angle_k * (factor_u + factor_v);

      // Thank God that is out of the way. Target velocity = (unit velocity + dunit velocity)
      tf2::Vector3 gamma_target = (gamma + dunit_gamma_target) * pursuit_velocity;
      pilot.set_targetvel(gamma_target);

      loop_rate.sleep();
    }

    return 0;
  }
  
  int takeoff(){
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

    mavros_msgs::CommandTOL command_payload;
    command_payload.request.altitude = 5.0;

    client.call(command_payload);

    return command_payload.response.result;
  }
}