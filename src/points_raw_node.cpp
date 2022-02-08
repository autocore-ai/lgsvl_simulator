#include <ros/ros.h>

#include "lgsvl_simulator/points_raw_proxy.h"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "points_raw_proxy");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  PointsRawProxy node(nh, private_nh);
  ros::spin();
  return 0;
}
