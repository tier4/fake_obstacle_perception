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
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/Point.h>

namespace rviz_plugins
{

class PointsMap
{
public:
  PointsMap();

  bool isSubscribed(){ return is_subscribed_; }
  std::pair<bool, double> requestPositionZ(const geometry_msgs::Point &point);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw_;
  pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree2d_;
  bool is_subscribed_;

  void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
};
}
