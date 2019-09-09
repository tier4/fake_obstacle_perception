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

#include "widget.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QValidator>
#include <QTimer>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QListWidget>
#include <QListWidgetItem>

#include <visualization_msgs/MarkerArray.h>

#include <yaml-cpp/yaml.h>

namespace rviz_plugins
{
FakeObstaclesPublisher::FakeObstaclesPublisher(QWidget* parent)
{
  ma_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("fake_obstacles_publisher/marker", 1);

  save_dir_ = QDir::homePath();

  createLayout();
}

void FakeObstaclesPublisher::update()
{
  // button operation
  topic_name_->setEnabled([this](){
    return !pub_button_->isChecked();
  }());

  // button operation
  density_->setEnabled([this](){
    return !pub_button_->isChecked();
  }());

  add_button_->setEnabled([this](){
    return !pub_button_->isChecked();
  }());

  remove_button_->setEnabled([this](){
    return !pub_button_->isChecked() && list_widget_->count() > 0;
  }());

  load_button_->setEnabled([this](){
    return !pub_button_->isChecked();
  }());

  save_button_->setEnabled([this](){
    return !pub_button_->isChecked() && list_widget_->count() > 0 && !topic_name_->text().isEmpty();
  }());
  pub_button_->setEnabled([this](){
    return list_widget_->count() > 0 && !topic_name_->text().isEmpty() && !density_->text().isEmpty();
  }());


  // publish marker
  visualization_msgs::MarkerArray ma;
  for(auto &e : obst_list_v_)
  {
    visualization_msgs::MarkerArray obst_ma;
    if(std::get<1>(e))
      obst_ma = std::get<2>(e).createMarkerArray();
    else
      obst_ma = std::get<2>(e).deleteMarkerArray();

    std::copy(obst_ma.markers.begin(), obst_ma.markers.end(), std::back_inserter(ma.markers));
  }
  ma_pub_.publish(ma);


  // move obstacle, publish pointcloud
  if(pub_button_->isChecked())
  {
    PointCloudXYZI::Ptr obst_pcs(new PointCloudXYZI());

    //ROS_INFO("obstacle size : %d", (int32_t)obst_list_v_.size());
    for(auto &e : obst_list_v_)
    {
      if(std::get<1>(e))
      {
        std::get<2>(e).update();
        *obst_pcs += std::get<2>(e).createShapePCL(density_->text().toInt());
      }
      else
      {
        std::get<2>(e).pause();
      }
    }

    std_msgs::Header header;
    header.frame_id = std::get<2>(obst_list_v_.front()).getTargetFrame();
    header.stamp = ros::Time::now();
    obst_pcs->header = pcl_conversions::toPCL(header);
    pcl_pub_ptr_->publish(obst_pcs);

  }
  else
  {
    for(auto &e: obst_list_v_)
      std::get<2>(e).reset();
  }

}

void FakeObstaclesPublisher::add()
{
  ROS_INFO_STREAM(__FUNCTION__);
  QStringList filepaths = QFileDialog::getOpenFileNames(this, "Load Fake Obstacle Settings", QDir::homePath(), tr("YAML (*.yaml)"));

  if (filepaths.isEmpty())
    return;

  for(const auto& e : filepaths)
  {
    addItem(e);
  }
}

void FakeObstaclesPublisher::load()
{
  QString filepath = QFileDialog::getOpenFileName(this, "Load Fake Obstacle Settings Preset", QDir::homePath(), tr("YAML (*.yaml)"));

  if (filepath.isEmpty())
    return;

  auto preset_tuple = parsePresetYAML(filepath.toUtf8().constData());
  if(std::get<0>(preset_tuple))
  {
    topic_name_->setText(QString::fromStdString(std::get<1>(preset_tuple)));
    density_->setText(QString::fromStdString(std::to_string(std::get<2>(preset_tuple))));
    for(const auto& e : std::get<3>(preset_tuple))
    {
      QString fp = QFileInfo(filepath).absolutePath() + "/" + QString::fromStdString(e);
      addItem(fp);
    }
  }
}

void FakeObstaclesPublisher::addItem(const QString &filepath)
{
  auto obst_pair = parseObstacleYAML(filepath.toUtf8().constData());
  if(obst_pair.first)
  {
    obst_list_v_.emplace_back(QFileInfo(filepath).fileName().toUtf8().constData(), false, obst_pair.second);

    auto item = new QListWidgetItem(QString::fromStdString(obst_pair.second.getMessage()), list_widget_);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
    item->setCheckState(Qt::Unchecked); // AND initialize check state
  }
  else
  {
    ROS_WARN("cannot read: %s", filepath.toUtf8().constData());
  }
}

void FakeObstaclesPublisher::remove()
{
  for(const auto & e : list_widget_->selectedItems())
    obst_list_v_.erase(obst_list_v_.begin() + list_widget_->row(e));

  qDeleteAll(list_widget_->selectedItems());
}

void FakeObstaclesPublisher::check(QListWidgetItem *item)
{
  ROS_INFO_STREAM(__FUNCTION__);
  if(obst_list_v_.empty())
    return;

  auto &elem = obst_list_v_.at(list_widget_->row(item));
  if(item->checkState() == Qt::Checked)
    std::get<1>(elem) = true;
  else
  {
    std::get<1>(elem) = false;
  }
}

void FakeObstaclesPublisher::save()
{
  QString filepath = QFileDialog::getSaveFileName(this, "Save Fake Obstacle Settings Preset", save_dir_, tr("YAML (*.yaml)"));

  if (filepath.isEmpty())
    return;

  filepath = [filepath](){
    std::string str = filepath.toUtf8().constData();
    if(str.find(".yaml") == std::string::npos)
      str += ".yaml";
    return QString::fromStdString(str);
  }();

  std::ofstream ofs(filepath.toUtf8().constData());

  // YAML
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;

  // set topic name
  emitter << YAML::Key << "topic_name";
  emitter << YAML::Value << topic_name_->text().toUtf8().constData();

  // set density
  emitter << YAML::Key << "density";
  emitter << YAML::Value << density_->text().toUtf8().constData();

  // set file list
  emitter << YAML::Key << "file_list";
  emitter << YAML::Value << YAML::BeginSeq;
  for(const auto &e : obst_list_v_)
  {
    emitter << std::get<0>(e);
  }
  emitter << YAML::EndSeq;


  emitter << YAML::EndMap;
  //std::cout << emitter.c_str() << std::endl;
  ofs << emitter.c_str();
  ofs.close();
  save_dir_ = QFileInfo(filepath).dir().path();

}

void FakeObstaclesPublisher::publish(bool checked)
{
  if(checked)
  {
    pcl_pub_ptr_.reset(new ros::Publisher());
    *pcl_pub_ptr_ = nh_.advertise<PointCloudXYZI>(topic_name_->text().toUtf8().constData(), 1);
  }
  else
  {
    pcl_pub_ptr_.release();
  }
}

void FakeObstaclesPublisher::createLayout()
{
  // Setup Layout

  // Topic Name
  auto topic_name_label = new QLabel(tr("Topic Name"));
  topic_name_ = new QLineEdit;
  topic_name_->setPlaceholderText("ex. /points_raw");
  auto tn_layout = new QHBoxLayout();
  tn_layout->addWidget(topic_name_label);
  tn_layout->addWidget(topic_name_);

  // density
  auto density_label = new QLabel(tr("Density"));
  density_ = new QLineEdit;
  density_->setValidator( new QIntValidator(0, 100, this));
  density_->setPlaceholderText("ex. 3");
  auto den_layout = new QHBoxLayout();
  den_layout->addWidget(density_label);
  den_layout->addWidget(density_);

  list_widget_ = new QListWidget;
  list_widget_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  connect(list_widget_, &QListWidget::itemChanged, this, &FakeObstaclesPublisher::check);

  // upper button layout
  add_button_ = new QPushButton("ADD");
  remove_button_ = new QPushButton("REMOVE");
  remove_button_->setEnabled(false);
  connect(add_button_, &QPushButton::clicked, this, &FakeObstaclesPublisher::add);
  connect(remove_button_, &QPushButton::clicked, this, &FakeObstaclesPublisher::remove);


  auto upper_button_layout = new QHBoxLayout();
  upper_button_layout->addWidget(add_button_);
  upper_button_layout->addWidget(remove_button_);

  //bottom button layout
  save_button_ = new QPushButton("SAVE");
  load_button_ = new QPushButton("LOAD");
  pub_button_ = new QPushButton("PUBLISH");
  pub_button_->setCheckable(true);
  connect(load_button_, &QPushButton::clicked, this, &FakeObstaclesPublisher::load);
  connect(save_button_, &QPushButton::clicked, this, &FakeObstaclesPublisher::save);
  connect(pub_button_, &QPushButton::clicked, this, &FakeObstaclesPublisher::publish);

  auto bottom_button_layout = new QHBoxLayout();
  bottom_button_layout->addWidget(save_button_);
  bottom_button_layout->addWidget(load_button_);
  bottom_button_layout->addWidget(pub_button_);

  auto objsl_layout = new QVBoxLayout();
  objsl_layout->addLayout(upper_button_layout);
  objsl_layout->addWidget(list_widget_);

  auto objsl_group = new QGroupBox(tr("Obstacles SetList"));
  objsl_group->setLayout(objsl_layout);

  log_view_ = new QPlainTextEdit();
  log_view_->setReadOnly(true);

  // integrate layout
  auto vlayout = new QVBoxLayout();
  vlayout->addLayout(tn_layout);
  vlayout->addLayout(den_layout);
  vlayout->addWidget(objsl_group);
  vlayout->addLayout(bottom_button_layout);
  //vlayout->addWidget(log_view_);
  setLayout(vlayout);

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &FakeObstaclesPublisher::update);
  timer->start(1000 / 30);
}


}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::FakeObstaclesPublisher, rviz::Panel)
