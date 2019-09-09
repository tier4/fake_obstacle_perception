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

#include "points_map.hpp"

#include <chrono>

// ROS includes
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/topic.h>

namespace rviz_plugins
{
PointsMap::PointsMap() : is_subscribed_(false)
{
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr msg = ros::topic::waitForMessage<pcl::PointCloud<pcl::PointXYZ>>("points_map", ros::Duration(0.5));
  callback(msg);
}

std::pair<bool, double> PointsMap::requestPositionZ(const geometry_msgs::Point &point)
{
  if(cloud_raw_ != nullptr && tree2d_ != nullptr)
  {
    //ROS_INFO("get z position");
    double radius = 1.0;
    std::vector<int32_t> k_indices;
    std::vector<float> k_sqr_distances;
    pcl::PointXY p;
    p.x = point.x;
    p.y = point.y;
    tree2d_->radiusSearch(p, radius, k_indices, k_sqr_distances, 5);

    if (!k_indices.empty())
    {
      double res = 0.0;
      for(const auto &e : k_indices)
      {
        //ROS_INFO("%d: %lf",e,  cloud_raw_->points.at(e).z);
        res += cloud_raw_->points.at(e).z;
      }
      res /= k_indices.size();
      //ROS_INFO("res: %lf", res);
      return std::make_pair(true, res);
    }
    else
    {
      ROS_WARN("cannot find point cloud...");
      return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
    }
  }
  else
  {
    ROS_WARN("not initialized");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }
}

void PointsMap::callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  if(msg == nullptr)
  {
    ROS_WARN("cannot subscribe points_map");
    return;
  }

  //ROS_INFO(__FUNCTION__);
  //ROS_INFO("is_subscribed_ : %s", is_pointcloud_subscribed_ != 0 ? "True": "False");

  cloud_raw_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  //auto proc_start1 = std::chrono::system_clock::now();

  // Delete NaN
  std::vector<int> nan_index;
  pcl::removeNaNFromPointCloud(*msg, *cloud_raw_, nan_index);

  /*auto proc_end1 = std::chrono::system_clock::now();
  double proc_elapsed1 = std::chrono::duration_cast<std::chrono::nanoseconds>(proc_end1 - proc_start1).count();
  ROS_INFO("processing time 1 : %lf [ms]", proc_elapsed1 * 1.0e-6);*/

  //auto proc_start4 = std::chrono::system_clock::now();
  // voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_raw_);
  sor.setLeafSize (0.2, 0.2, 0.2);
  sor.filter (*cloud_raw_);
  /*auto proc_end4 = std::chrono::system_clock::now();
  double proc_elapsed4 = std::chrono::duration_cast<std::chrono::nanoseconds>(proc_end4 - proc_start4).count();
  ROS_INFO("processing time 4 : %lf [ms]", proc_elapsed4 * 1.0e-6);*/

  //auto proc_start2 = std::chrono::system_clock::now();
  // create tree 2d
  pcl::PointCloud<pcl::PointXY>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXY>);
  cloud2d->points.resize(cloud_raw_->size());
  //ROS_INFO("size: %d", (int32_t)cloud_raw_->points.size());

  for (uint32_t i = 0; i < cloud_raw_->points.size(); i++)
  {
    cloud2d->points.at(i).x = cloud_raw_->points.at(i).x;
    cloud2d->points.at(i).y = cloud_raw_->points.at(i).y;
  }

  /*auto proc_end2 = std::chrono::system_clock::now();
  double proc_elapsed2 = std::chrono::duration_cast<std::chrono::nanoseconds>(proc_end2 - proc_start2).count();
  ROS_INFO("processing time 2 : %lf [ms]", proc_elapsed2 * 1.0e-6);*/

  //auto proc_start3 = std::chrono::system_clock::now();
  tree2d_.reset(new pcl::KdTreeFLANN<pcl::PointXY>);
  tree2d_->setInputCloud(cloud2d);

  /*auto proc_end3 = std::chrono::system_clock::now();
  double proc_elapsed3 = std::chrono::duration_cast<std::chrono::nanoseconds>(proc_end3 - proc_start3).count();
  ROS_INFO("processing time 3 : %lf [ms]", proc_elapsed3 * 1.0e-6);*/
  is_subscribed_ = true;
}
}
