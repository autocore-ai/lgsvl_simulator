/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lgsvl_simulator/odom_to_twist_core.h"

OdomToTwist::OdomToTwist(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh), output_frame_("base_link")
{
  private_nh_.getParam("output_frame", output_frame_);
  odom_sub_ = nh_.subscribe("odom", 100, &OdomToTwist::callbackOdom, this);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vehicle/twist", 10);
}

OdomToTwist::~OdomToTwist() {}

void OdomToTwist::callbackOdom(const nav_msgs::Odometry::ConstPtr & odom_msg_ptr)
{
  geometry_msgs::TwistStamped ts;
  ts.header.frame_id = output_frame_;
  ts.header.seq = odom_msg_ptr->header.seq;
  // ts.header.stamp = odom_msg_ptr->header.stamp;
  ts.header.stamp = ros::Time::now();
  ts.twist = odom_msg_ptr->twist.twist;
  twist_pub_.publish(ts);
}
