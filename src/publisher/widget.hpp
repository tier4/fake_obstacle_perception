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
#include <rviz/panel.h>


#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QListWidget>

#include <vector>
#include <memory>

#include "../utils/obstacle.hpp"
#include "../utils/waypoints.hpp"
#include "../utils/read_yaml.hpp"

namespace rviz_plugins
{

typedef std::vector<std::tuple<std::string, bool, Obstacle>> ObstacleList;

class FakeObstaclesPublisher : public rviz::Panel
{
  Q_OBJECT

public:
  FakeObstaclesPublisher(QWidget* parent = 0);

private:
  ObstacleList obst_list_v_;
  QString save_dir_;
  QPlainTextEdit* log_view_;
  ros::NodeHandle nh_;
  std::unique_ptr<ros::Publisher> pcl_pub_ptr_;
  ros::Publisher ma_pub_;

  //parameter
  QLineEdit *topic_name_;
  QLineEdit *density_;
  QPushButton* pub_button_;
  QPushButton* add_button_;
  QPushButton* save_button_;
  QPushButton* remove_button_;
  QPushButton* load_button_;
  QListWidget* list_widget_;


  void createLayout();
  void addItem(const QString &filepath);

private Q_SLOTS:

  void update();
  void add();
  void remove();
  void check(QListWidgetItem *item);

  void load();
  void save();
  void publish(bool checked);



};

}
