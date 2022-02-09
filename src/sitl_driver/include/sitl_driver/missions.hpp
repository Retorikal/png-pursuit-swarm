#include <ros/ros.h>
#pragma once

class sitl_driver::Pilot;

namespace missions{
  int png_chase(sitl_driver::Pilot* pilot);
  int takeoff();
}