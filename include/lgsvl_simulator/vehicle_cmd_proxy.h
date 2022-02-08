#pragma once

#include <ros/ros.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>

class VehicleCmdProxy
{
public:
  VehicleCmdProxy(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~VehicleCmdProxy();

private:
  void callbackVehicleCmd(const autoware_vehicle_msgs::VehicleCommandConstPtr & msg);
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
};
