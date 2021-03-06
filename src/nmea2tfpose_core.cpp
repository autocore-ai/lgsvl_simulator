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

#include "lgsvl_simulator/nmea2tfpose_core.h"

namespace gnss_localizer
{
// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(-std::numeric_limits<double>::infinity())
  , position_time_(-std::numeric_limits<double>::infinity())
  , current_time_(0)
  , orientation_stamp_(0)
  , orientation_ready_(false)
{
  initForROS();
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter settings
  private_nh_.getParam("offset_x", offset_x);
  private_nh_.getParam("offset_y", offset_y);
  private_nh_.getParam("offset_z", offset_z);
  private_nh_.getParam("offset_roll", offset_roll);
  private_nh_.getParam("offset_pitch", offset_pitch);
  private_nh_.getParam("offset_yaw", offset_yaw);

  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("gnss_pose_cov", 10);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseWithCovarianceStamped()
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.pose.position.x = geo_.y() + offset_x;
  pose.pose.pose.position.y = geo_.x() + offset_y;
  pose.pose.pose.position.z = geo_.z() + offset_z;
  pose.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_ + offset_roll, pitch_ + offset_pitch, yaw_ + offset_yaw);
  pub1_.publish(pose);
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.y() + offset_x, geo_.x() + offset_y, geo_.z() + offset_z));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_ + offset_roll, pitch_ + offset_pitch, yaw_ + offset_yaw);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  roll_ = 0;
  pitch_ = 0;
}

void Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
  try
  {
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      yaw_ = -1 * stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = 1 * stod(nmea.at(5)) * M_PI / 180.;
      roll_ = 1 * stod(nmea.at(6)) * M_PI / 180.;
      orientation_stamp_ = current_stamp;
      orientation_ready_ = true;
      // ROS_INFO("QQ is subscribed.%f:%f:%f",roll_,pitch_,yaw_);
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      orientation_ready_ = true;
      ROS_INFO("PASHR is subscribed.");
    }
    else if (nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));

      if (nmea.at(3) == "S")
        lat = -lat;

      if (nmea.at(5) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);
      //ROS_INFO("GGA is subscribed.%f:%f",lat,lon);
    }
    else if (nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;

      if (nmea.at(4) == "S")
        lat = -lat;

      if (nmea.at(6) == "W")
        lon = -lon;

      geo_.set_llh_nmea_degrees(lat, lon, h);

      ROS_INFO("GPRMC is subscribed.");
    }
  }
  catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  // current_time_ = msg->header.stamp;
  current_time_ = ros::Time::now();
  convert(split(msg->sentence), msg->header.stamp);

  double timeout = 10.0;
  // if orientation_stamp_ is 0 then no "QQ" sentence was ever received,
  // so orientation should be computed from offsets
  if (orientation_stamp_.isZero()
      || fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_.x(), 2) + pow(geo_.y() - last_geo_.y(), 2));
    double threshold = 0.2;
    if (dt > threshold)
    {
      /* If orientation data is not available it is generated based on translation
         from the previous position. For the first message the previous position is
         simply the origin, which gives a wildly incorrect orientation. Some nodes
         (e.g. ndt_matching) rely on that first message to initialise their pose guess,
         and cannot recover from such incorrect orientation.
         Therefore the first message is not published, ensuring that orientation is
         only calculated from sensible positions.
      */
      if (orientation_ready_)
      {
        ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
        createOrientation();
        publishPoseWithCovarianceStamped();
        publishTF();
      }
      else
      {
        orientation_ready_ = true;
      }
      last_geo_ = geo_;
    }
    return;
  }

  double e = 1e-2;
  if ((fabs(orientation_time_ - position_time_) < e) && orientation_ready_)
  {
    publishPoseWithCovarianceStamped();
    publishTF();
    return;
  }
}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

}  // namespace gnss_localizer
