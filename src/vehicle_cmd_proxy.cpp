#include "lgsvl_simulator/vehicle_cmd_proxy.h"
#include "autoware_msgs/VehicleCmd.h"

VehicleCmdProxy::VehicleCmdProxy(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh)
{
  sub_ = nh_.subscribe("input", 1, &VehicleCmdProxy::callbackVehicleCmd, this);
  pub_ = nh_.advertise<autoware_msgs::VehicleCmd>("output", 10);
}

VehicleCmdProxy::~VehicleCmdProxy() {}

void VehicleCmdProxy::callbackVehicleCmd(const autoware_vehicle_msgs::VehicleCommandConstPtr & msg)
{
  autoware_msgs::VehicleCmd data;
  data.header = msg->header;
  data.accel_cmd.accel = msg->control.acceleration;
  data.steer_cmd.steer = msg->control.steering_angle;
  data.twist_cmd.twist.linear.x = msg->control.velocity;
  data.twist_cmd.twist.angular.z = msg->control.steering_angle * 3.14;
  data.ctrl_cmd.linear_velocity = msg->control.velocity;
  data.ctrl_cmd.linear_acceleration = msg->control.acceleration;
  data.ctrl_cmd.steering_angle = msg->control.steering_angle;
  pub_.publish(data);
}
