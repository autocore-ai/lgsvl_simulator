#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class IMURawProxy
{
public:
  IMURawProxy(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~IMURawProxy();

private:
  void callback(const sensor_msgs::Imu::ConstPtr & msg);
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};
