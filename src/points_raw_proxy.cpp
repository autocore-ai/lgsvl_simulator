#include "lgsvl_simulator/points_raw_proxy.h"

PointsRawProxy::PointsRawProxy(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{
  sub_ = nh_.subscribe("input", 1, &PointsRawProxy::callback, this);
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 10);
}

PointsRawProxy::~PointsRawProxy() {}

void PointsRawProxy::callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  sensor_msgs::PointCloud2 data = *msg;
  data.header.stamp = ros::Time::now();
  pub_.publish(data);
}
