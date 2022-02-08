#include "lgsvl_simulator/imu_raw_proxy.h"

IMURawProxy::IMURawProxy(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{
  sub_ = nh_.subscribe("input", 1, &IMURawProxy::callback, this);
  pub_ = nh_.advertise<sensor_msgs::Imu>("output", 10);
}

IMURawProxy::~IMURawProxy() {}

void IMURawProxy::callback(const sensor_msgs::Imu::ConstPtr & msg)
{
  sensor_msgs::Imu data = *msg;
  data.header.stamp = ros::Time::now();
  pub_.publish(data);
}
