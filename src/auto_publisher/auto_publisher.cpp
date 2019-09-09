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

#include "auto_publisher.h"

#include <visualization_msgs/MarkerArray.h>

#include <fstream>
#include <yaml-cpp/yaml.h>


namespace rviz_plugins
{
FakeObstaclesAutoPublisher::FakeObstaclesAutoPublisher() : pnh_("~")
{
  pnh_.param<double>("execution_period", exec_period_, 0.04);
  pnh_.param<std::string>("preset_path", preset_path_, "test.yaml");

  initializeObstacles();

  // setup processing timer
  proc_timer_ = nh_.createTimer(ros::Duration(exec_period_), &FakeObstaclesAutoPublisher::timerCallback, this);

  // publisher
  ma_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker", 1);
  pcl_pub_ = nh_.advertise<PointCloudXYZI>(publish_topic_name_, 1);

}

void FakeObstaclesAutoPublisher::timerCallback(const ros::TimerEvent &event)
{
  ROS_DEBUG("processing...");
  double diff_expected = event.current_expected.toSec() - event.last_expected.toSec();
  double diff_real = event.current_real.toSec() - event.last_real.toSec();
  ROS_DEBUG("diff_expected: %lf, diff_real: %lf", diff_expected, diff_real);

  auto proc_start = std::chrono::system_clock::now();

  auto proc_end = std::chrono::system_clock::now();
  double proc_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(proc_end - proc_start).count();
  ROS_DEBUG("processing time : %lf [ms]", proc_elapsed * 1.0e-6);

  // publish marker
  visualization_msgs::MarkerArray ma;
  for(auto &e : obst_v_)
  {
    visualization_msgs::MarkerArray obst_ma = e.createMarkerArray();

    std::copy(obst_ma.markers.begin(), obst_ma.markers.end(), std::back_inserter(ma.markers));
  }
  ma_pub_.publish(ma);


  // move obstacle, publish pointcloud

  PointCloudXYZI::Ptr obst_pcs(new PointCloudXYZI());

  ROS_INFO("obstacle size : %d", (int32_t)obst_v_.size());
  for(auto &e : obst_v_)
  {
    e.update();
    *obst_pcs += e.createShapePCL(density_);
  }

  std_msgs::Header header;
  header.frame_id = obst_v_.front().getTargetFrame();
  header.stamp = ros::Time::now();
  obst_pcs->header = pcl_conversions::toPCL(header);
  pcl_pub_.publish(obst_pcs);

}


void FakeObstaclesAutoPublisher::initializeObstacles()
{
  auto preset_tuple = parsePresetYAML(preset_path_);
  if(std::get<0>(preset_tuple))
  {
    publish_topic_name_ = std::get<1>(preset_tuple);
    density_ = std::get<2>(preset_tuple);

    for(const auto& e : std::get<3>(preset_tuple))
    {
      std::string fp = getAbsolutePath(preset_path_) + "/" + e;
      addItem(fp);
    }
  }
}

void FakeObstaclesAutoPublisher::addItem(const std::string &filepath)
{
  auto obst_pair = parseObstacleYAML(filepath);
  if(obst_pair.first)
  {
    obst_v_.emplace_back(obst_pair.second);
  }
  else
  {
    ROS_WARN("cannot read: %s", filepath.c_str());
  }
}


std::string getAbsolutePath(const std::string &filepath)
{
  std::size_t found = filepath.find_last_of("/\\");
  return filepath.substr(0,found);
}

}