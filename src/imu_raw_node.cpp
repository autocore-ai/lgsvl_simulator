#include <ros/ros.h>

#include "lgsvl_simulator/imu_raw_proxy.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "imu_raw_proxy");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  IMURawProxy node(nh, private_nh);
  ros::spin();
  return 0;
}
