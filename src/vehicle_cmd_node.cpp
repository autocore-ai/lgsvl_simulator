#include <ros/ros.h>

#include "lgsvl_simulator/vehicle_cmd_proxy.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "vechicle_cmd_proxy");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  VehicleCmdProxy node(nh, private_nh);
  ros::spin();
  return 0;
}
