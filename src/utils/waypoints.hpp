//
//  Copyright 2019 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <memory>


namespace rviz_plugins
{

class Waypoints
{
public:
  explicit Waypoints(const std::string &node_name) : selected_(wps_raw_.end()), NS_RAW("raw"), NS_INTERPOLATED("spline")
  {
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(node_name + "/marker", 1);
  };

  void add(const geometry_msgs::PointStamped &ps);
  void erase(const geometry_msgs::PointStamped &ps);
  void move(const geometry_msgs::PointStamped &ps);
  void select(const geometry_msgs::PointStamped &ps);
  void release();
  void setTargetFrame(const std::string tf) { target_frame_ = tf; }
  std::vector<geometry_msgs::Pose> getWaypointsRaw() { return wps_raw_; }
  std::vector<geometry_msgs::Pose> getWaypointsInterpolated() { return wps_intp_; }
  void reset();
  void load(const std::vector<geometry_msgs::Pose> &wps_intp, const std::vector<geometry_msgs::Pose> &wps_raw);
  void publishMarker();
  void publishDeleteMarker();
  uint32_t getSize(){ return wps_raw_.size(); }
  
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string target_frame_;
  std::vector<geometry_msgs::Pose> wps_intp_;
  std::vector<geometry_msgs::Pose> wps_raw_;
  std::vector<geometry_msgs::Pose>::iterator selected_;
  const std::string NS_RAW;
  const std::string NS_INTERPOLATED;

  void angleInterpolation();
};
}
