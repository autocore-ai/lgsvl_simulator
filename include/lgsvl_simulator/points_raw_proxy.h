#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class PointsRawProxy
{
public:
  PointsRawProxy(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~PointsRawProxy();

private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr & msg);
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};
