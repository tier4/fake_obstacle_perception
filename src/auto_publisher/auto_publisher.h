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

#include <vector>
#include <memory>

#include "../utils/obstacle.hpp"
#include "../utils/waypoints.hpp"
#include "../utils/read_yaml.hpp"

namespace rviz_plugins
{

class FakeObstaclesAutoPublisher
{
public:
  FakeObstaclesAutoPublisher();

private:
  std::vector<Obstacle> obst_v_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher ma_pub_, pcl_pub_;
  std::string publish_topic_name_, preset_path_;
  double exec_period_;
  ros::Timer proc_timer_;
  int32_t density_;

  void initializeObstacles();
  void timerCallback(const ros::TimerEvent &event);
  void addItem(const std::string &filepath);

};

std::string getAbsolutePath(const std::string &filepath);

}
