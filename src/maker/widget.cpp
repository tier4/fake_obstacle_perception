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
#include <QApplication>


#include <fstream>
#include <yaml-cpp/yaml.h>
#include <chrono>

// ROS includes
#include <pcl/filters/filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/voxel_grid.h>

namespace rviz_plugins
{
FakeObstacleMaker::FakeObstacleMaker(QWidget* parent)
    : wps_(Waypoints("fake_obstacle_maker"))
    , tf2_listener_(tf2_buffer_)
    , prev_left_button_(false)
    , prev_right_button_(false)
{
  ec_sub_ = nh_.subscribe("/rviz/event_capture/mouse", 1, &rviz_plugins::FakeObstacleMaker::callback, this);

  save_dir_ = QDir::homePath();

  createLayout();
}

void FakeObstacleMaker::update()
{
  // update target frame
  wps_.setTargetFrame(target_frame_->text().toUtf8().constData());
  //wps_.publishDeleteMarker();
  wps_.publishMarker();


  edit_button_->setEnabled([this](){
    return !test_button_->isChecked();
  }());

  load_button_->setEnabled([this](){
    return edit_button_->isChecked();
  }());

  reset_button_->setEnabled([this](){
    return edit_button_->isChecked();
  }());

  save_button_->setEnabled([this](){
    return !target_frame_->text().isEmpty() && !width_->text().isEmpty() && !length_->text().isEmpty()
        && !height_->text().isEmpty() && !velocity_->text().isEmpty() && !intensity_->text().isEmpty()
        && !lifetime_->text().isEmpty() && !respawn_delay_->text().isEmpty() && wps_.getSize() > 2
        && !edit_button_->isChecked() && !test_button_->isChecked();
  }());

  test_button_->setEnabled([this](){
    return !target_frame_->text().isEmpty() && !width_->text().isEmpty() && !length_->text().isEmpty()
        && !height_->text().isEmpty() && !velocity_->text().isEmpty() && !intensity_->text().isEmpty()
        && !lifetime_->text().isEmpty() && !respawn_delay_->text().isEmpty() && wps_.getSize() > 2
        && !edit_button_->isChecked();
  }());

  // test mode
  if(test_button_->isChecked() && obst_ptr_ != nullptr)
    obst_ptr_->update();
}

void FakeObstacleMaker::edit(bool checked)
{
  if(checked)
  {
    target_frame_->setEnabled(true);
    obj_info_group_->setEnabled(true);
  }
  else
  {
    target_frame_->setEnabled(false);
    obj_info_group_->setEnabled(false);
  }

}

void FakeObstacleMaker::load()
{
  QString filepath = QFileDialog::getOpenFileName(this, "Load Fake Obstacle Settings", QDir::homePath(), tr("YAML (*.yaml)"));

  if (filepath.isEmpty())
    return;

  try
  {
    YAML::Node node = YAML::LoadFile(filepath.toUtf8().constData());
    //for(const auto &n : node)
    //  ROS_INFO_STREAM(n.first << ": "  << n.second);

    target_frame_->setText(QString::fromStdString(node["target_frame"].as<std::string>()));
    width_->setText(QString::number(node["scale"]["width"].as<double>()));
    length_->setText(QString::number(node["scale"]["length"].as<double>()));
    height_->setText(QString::number(node["scale"]["height"].as<double>()));
    velocity_->setText(QString::number(node["velocity"].as<double>()));
    intensity_->setText(QString::number(node["intensity"].as<double>()));
    lifetime_->setText(QString::number(node["lifetime"].as<double>()));
    respawn_delay_->setText(QString::number(node["respawn_delay"].as<double>()));
    msg_->setText(QString::fromStdString(node["message"].as<std::string>()));

    std::vector<geometry_msgs::Pose> wps_intp;
    for(const auto& w : node["waypoints"])
    {
      geometry_msgs::Pose pose;
      pose.position.x = w["px"].as<double>();
      pose.position.y = w["py"].as<double>();
      pose.position.z = w["pz"].as<double>();
      pose.orientation.x = w["ox"].as<double>();
      pose.orientation.y = w["oy"].as<double>();
      pose.orientation.z = w["oz"].as<double>();
      pose.orientation.w = w["ow"].as<double>();
      wps_intp.push_back(pose);
    }

    std::vector<geometry_msgs::Pose> wps_raw;
    for(const auto& w : node["waypoints_raw"])
    {
      geometry_msgs::Pose pose;
      pose.position.x = w["px"].as<double>();
      pose.position.y = w["py"].as<double>();
      pose.position.z = w["pz"].as<double>();
      pose.orientation.x = w["ox"].as<double>();
      pose.orientation.y = w["oy"].as<double>();
      pose.orientation.z = w["oz"].as<double>();
      pose.orientation.w = w["ow"].as<double>();
      wps_raw.push_back(pose);
    }

    wps_.setTargetFrame(target_frame_->text().toUtf8().constData());
    wps_.load(wps_intp, wps_raw);
  }
  catch(YAML::Exception& e) {
    ROS_WARN_STREAM(e.what());
    //log_view_->insertPlainText(QString::fromStdString(std::string("Failure " + (std::string)e.what() + "\n")));
    //log_view_->moveCursor(QTextCursor::End);
  }
  catch(std::out_of_range &e)
  {
    ROS_WARN_STREAM(e.what());
    //log_view_->insertPlainText(QString::fromStdString(std::string("Failure " + (std::string)e.what() + "\n")));
    //log_view_->moveCursor(QTextCursor::End);
  }

  ROS_INFO("load: %s", filepath.toUtf8().constData());
  //log_view_->insertPlainText(QString::fromStdString(std::string("load: " + (std::string)filepath.toUtf8().constData() + "\n")));
  //log_view_->moveCursor(QTextCursor::End);
}

void FakeObstacleMaker::reset()
{
  auto result = QMessageBox::question(this, "Reset", "Reset obstacle information?", QMessageBox::Yes | QMessageBox::No);
  if (result == QMessageBox::Yes)
  {
    width_->setText("");
    length_->setText("");
    height_->setText("");
    velocity_->setText("");
    intensity_->setText("");
    lifetime_->setText("");
    respawn_delay_->setText("");
    msg_->setText("");
    target_frame_->setText("");
    wps_.reset();
  }

  //log_view_->insertPlainText(QString::fromStdString(std::string("reset \n")));
  //log_view_->moveCursor(QTextCursor::End);
}

void FakeObstacleMaker::save()
{
  QString filepath = QFileDialog::getSaveFileName(this, "Save Fake Obstacle Settings", save_dir_, tr("YAML (*.yaml)"));

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

  // set target frame
  emitter << YAML::Key << "target_frame";
  emitter << YAML::Value << target_frame_->text().toUtf8().constData();

  // set scale
  emitter << YAML::Key << "scale";
  emitter << YAML::Value;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "width" << YAML::Value << width_->text().toUtf8().constData();
  emitter << YAML::Key << "length" << YAML::Value << length_->text().toUtf8().constData();
  emitter << YAML::Key << "height" << YAML::Value << height_->text().toUtf8().constData();
  emitter << YAML::EndMap;

  // set velocity
  emitter << YAML::Key << "velocity";
  emitter << YAML::Value << velocity_->text().toUtf8().constData();
  emitter << YAML::Comment("(km/h)");

  // set intensity
  emitter << YAML::Key << "intensity";
  emitter << YAML::Value << intensity_->text().toUtf8().constData();
  emitter << YAML::Comment("(0~255)");

  // set lifetime
  emitter << YAML::Key << "lifetime";
  emitter << YAML::Value << lifetime_->text().toUtf8().constData();
  emitter << YAML::Comment(" -1 is invalid value (s)");

  // set respawn delay
  emitter << YAML::Key << "respawn_delay";
  emitter << YAML::Value << respawn_delay_->text().toUtf8().constData();
  emitter << YAML::Comment(" -1 is invalid value (s)");

  // set message
  emitter << YAML::Key << "message";
  emitter << YAML::Value << msg_->text().toUtf8().constData();

  // set waypoints
  emitter << YAML::Key << "waypoints";
  emitter << YAML::Value << YAML::BeginSeq;
  for(const auto &e : wps_.getWaypointsInterpolated())
  {
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "px" << YAML::Value << e.position.x;
    emitter << YAML::Key << "py" << YAML::Value << e.position.y;
    emitter << YAML::Key << "pz" << YAML::Value << e.position.z;
    emitter << YAML::Key << "ox" << YAML::Value << e.orientation.x;
    emitter << YAML::Key << "oy" << YAML::Value << e.orientation.y;
    emitter << YAML::Key << "oz" << YAML::Value << e.orientation.z;
    emitter << YAML::Key << "ow" << YAML::Value << e.orientation.w;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;

  // set waypoints raw
  emitter << YAML::Key << "waypoints_raw";
  emitter << YAML::Value << YAML::BeginSeq;
  for(const auto &e : wps_.getWaypointsRaw())
  {
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "px" << YAML::Value << e.position.x;
    emitter << YAML::Key << "py" << YAML::Value << e.position.y;
    emitter << YAML::Key << "pz" << YAML::Value << e.position.z;
    emitter << YAML::Key << "ox" << YAML::Value << e.orientation.x;
    emitter << YAML::Key << "oy" << YAML::Value << e.orientation.y;
    emitter << YAML::Key << "oz" << YAML::Value << e.orientation.z;
    emitter << YAML::Key << "ow" << YAML::Value << e.orientation.w;
    emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;

  emitter << YAML::EndMap;
  //std::cout << emitter.c_str() << std::endl;
  ofs << emitter.c_str();
  ofs.close();
  save_dir_ = QFileInfo(filepath).dir().path();

  //log_view_->insertPlainText(QString::fromStdString(std::string("saved:" + (std::string)filepath.toUtf8().constData() + "\n")));
  //log_view_->moveCursor(QTextCursor::End);

}

void FakeObstacleMaker::test(bool checked)
{
  if(checked)
  {
    obst_ptr_.reset(new Obstacle("fake_obstacle_maker"));
    obst_ptr_->setScale(std::stod(width_->text().toUtf8().constData()),
                        std::stod(length_->text().toUtf8().constData()),
                        std::stod(height_->text().toUtf8().constData()));
    obst_ptr_->setOthers(std::stod(velocity_->text().toUtf8().constData()),
                         std::stod(intensity_->text().toUtf8().constData()),
                         std::stod(lifetime_->text().toUtf8().constData()),
                         std::stod(respawn_delay_->text().toUtf8().constData()));
    obst_ptr_->setMessage(msg_->text().toUtf8().constData());
    obst_ptr_->setWaypoints(wps_.getWaypointsInterpolated());
    obst_ptr_->needVisualization(true, target_frame_->text().toUtf8().constData(), 5);
    obst_ptr_->update();
    //log_view_->insertPlainText(QString::fromStdString(std::string("test start\n")));
    //log_view_->moveCursor(QTextCursor::End);

  }
  else
  {
    obst_ptr_.release();
    //log_view_->insertPlainText(QString::fromStdString(std::string("test end\n")));
    //log_view_->moveCursor(QTextCursor::End);
  }


}

void FakeObstacleMaker::check(int state)
{
  if(state == Qt::Unchecked)
  {
    pm_ptr_.release();
    check_z_label_->setText(": Not Initialized");
  }
  else if(state == Qt::Checked)
  {
    auto result = QMessageBox::question(this, "Subsribe /points_map", "get z from /points_map?\n already created waypoints is reinitialized.", QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      wps_.reset();
    }
    check_z_label_->setText(": Initializing...");
    QApplication::processEvents();
    pm_ptr_.reset(new PointsMap());
    if(pm_ptr_->isSubscribed())
    {
      check_z_label_->setText(": Initialized");
    }
    else
    {
      check_z_label_->setText(": Initialize Failed");
      check_z_->setCheckState(Qt::Unchecked);
    }
  }
}

void FakeObstacleMaker::callback(const event_capture::MouseEventCaptureStampedConstPtr &msg)
{
  //ROS_INFO_STREAM(__FUNCTION__);
  if(!edit_button_->isChecked())
    return;

  auto ps_in = [msg]() {
    geometry_msgs::PointStamped ps;
    ps.header = msg->header;
    ps.point.x = msg->capture.ray.origin.x;
    ps.point.y = msg->capture.ray.origin.y;
    return ps;
  }();

  auto transformPointStamped = [this, msg, &ps_in]() -> std::pair<bool, geometry_msgs::PointStamped> {

    geometry_msgs::PointStamped ps_out;

    try
    {
      tf2_buffer_.transform(ps_in, ps_out, target_frame_->text().toUtf8().constData(), ros::Duration(0.0));

      /*ROS_INFO("in(x:%lf y:%lf z:%lf) -> out(x:%lf y:%lf z:%lf)",
               ps_in.point.x,
               ps_in.point.y,
               ps_in.point.z,
               ps_out.point.x,
               ps_out.point.y,
               ps_out.point.z);*/

      // get z value from pointcloud
      //ROS_INFO("pcl_sub: %s, check: %s", (is_pointcloud_subscribed_ != 0) ? "True" : "False", (check_z_->isChecked() != 0) ? "True": "False");
      if(pm_ptr_ != nullptr)
      {
        auto res_pair = pm_ptr_->requestPositionZ(ps_out.point);
        if(res_pair.first)
          ps_out.point.z = res_pair.second;
      }

      //ROS_INFO("end");
      return std::make_pair(true, ps_out);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN_STREAM(e.what());
      // Print exception which was caught
      //log_view_->insertPlainText(QString::fromStdString(std::string("Failure " + (std::string)ex.what() + "\n")));
      //log_view_->moveCursor(QTextCursor::End);
      return std::make_pair(false, geometry_msgs::PointStamped());
    }
  };

  //ROS_INFO("shift: %s", msg->capture.shift ? "True" : "False");
  //ROS_INFO("prev left: %s", prev_left_button_ ? "True" : "False");
  //ROS_INFO("left: %s", msg->capture.left ? "True" : "False");
  //ROS_INFO("prev right: %s", prev_right_button_ ? "True" : "False");
  //ROS_INFO("right: %s", msg->capture.right ? "True" : "False");


  if (!msg->capture.shift)
  {
    if (!prev_left_button_ && msg->capture.left) // left down
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("add: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_.add(ps_out_pair.second);
      }

      //log_view_->insertPlainText(QString::fromStdString(
      //    std::string("add: " + std::to_string(ps_out.point.x) + ", " + std::to_string(ps_out.point.y) + "\n")
       //   ));
      //log_view_->moveCursor(QTextCursor::End);
    }
    else if (!prev_right_button_ && msg->capture.right) // right down
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("erase: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_.erase(ps_out_pair.second);
      }
      //log_view_->insertPlainText(QString::fromStdString(
      //    std::string("erase: " + std::to_string(ps_out.point.x) + ", " + std::to_string(ps_out.point.y) + "\n")
      //));
      //log_view_->moveCursor(QTextCursor::End);
    }
  }
  else
  {
    // move
    if(!prev_left_button_ && msg->capture.left) // left down
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("select: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_.select(ps_out_pair.second);
      }
      //log_view_->insertPlainText(QString::fromStdString(
      //    std::string("move: " + std::to_string(ps_out.point.x) + ", " + std::to_string(ps_out.point.y) + ", " + std::to_string(ps_out.point.z) + "\n")
      //));
      //log_view_->moveCursor(QTextCursor::End);
    }
    else if(prev_left_button_ && msg->capture.left) // left keep
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("move: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_.move(ps_out_pair.second);
      }

    }
    else if(prev_left_button_ && !msg->capture.left) // left up
    {
      auto ps_out_pair = transformPointStamped();
      if(ps_out_pair.first)
      {
        ROS_INFO("release: %lf, %lf, %lf", ps_out_pair.second.point.x, ps_out_pair.second.point.y, ps_out_pair.second.point.z);
        wps_.release();
      }
    }
  }
  prev_left_button_ = msg->capture.left;
  prev_right_button_ = msg->capture.right;
}



void FakeObstacleMaker::createLayout()
{
  // Setup Layout

  // Target Frame
  auto target_frame_label = new QLabel(tr("TargetFrame"));
  target_frame_ = new QLineEdit;
  target_frame_->setPlaceholderText("ex. world");
  target_frame_->setEnabled(false);
  auto tf_layout = new QHBoxLayout();
  tf_layout->addWidget(target_frame_label);
  tf_layout->addWidget(target_frame_);

  // Scale parameter
  auto width_label = new QLabel(tr("Width:"));
  auto length_label = new QLabel(tr("Length:"));
  auto height_label = new QLabel(tr("Height:"));
  width_ = new QLineEdit;
  width_->setPlaceholderText("[m]");
  width_->setValidator( new QDoubleValidator(0, 100, 2, this));
  length_ = new QLineEdit;
  length_->setPlaceholderText("[m]");
  length_->setValidator( new QDoubleValidator(0, 100, 2, this));
  height_ = new QLineEdit;
  height_->setPlaceholderText("[m]");
  height_->setValidator( new QDoubleValidator(0, 100, 2, this));
  auto scale_layout = new QGridLayout;
  scale_layout->addWidget(width_label, 0, 0);
  scale_layout->addWidget(width_, 0, 1);
  scale_layout->addWidget(length_label, 1, 0);
  scale_layout->addWidget(length_, 1, 1);
  scale_layout->addWidget(height_label, 2, 0);
  scale_layout->addWidget(height_, 2, 1);
  auto scale_group = new QGroupBox(tr("Scale"));
  scale_group->setLayout(scale_layout);

  // Other parameter
  auto velocity_label = new QLabel(tr("Velocity :"));
  auto intensity_label = new QLabel(tr("Intensity (0~255):"));
  auto lifetime_label = new QLabel(tr("Lifetime:"));
  auto respawn_delay_label = new QLabel(tr("Respawn Delay:"));
  velocity_ = new QLineEdit;
  velocity_->setPlaceholderText("[km/h]");
  velocity_->setValidator( new QDoubleValidator(0, 100, 2, this));
  intensity_ = new QLineEdit;
  intensity_->setPlaceholderText("[0 ~ 255]");
  intensity_->setValidator( new QIntValidator(0, 255, this));
  lifetime_ = new QLineEdit;
  lifetime_->setPlaceholderText("-1 is infinity");
  lifetime_->setValidator( new QDoubleValidator(-1.0, 100000, 2, this));
  respawn_delay_ = new QLineEdit;
  respawn_delay_->setPlaceholderText("-1 is no time");
  respawn_delay_->setValidator( new QDoubleValidator(-1.0, 100000, 2, this));
  auto others_layout = new QGridLayout;
  others_layout->addWidget(velocity_label, 0, 0);
  others_layout->addWidget(velocity_, 0, 1);
  others_layout->addWidget(intensity_label, 1, 0);
  others_layout->addWidget(intensity_, 1, 1);
  others_layout->addWidget(lifetime_label, 2, 0);
  others_layout->addWidget(lifetime_, 2, 1);
  others_layout->addWidget(respawn_delay_label, 3, 0);
  others_layout->addWidget(respawn_delay_, 3, 1);
  auto others_group = new QGroupBox(tr("Others"));
  others_group->setLayout(others_layout);

  // parameter layout
  auto param_layout = new QGridLayout;
  param_layout->addWidget(scale_group, 0, 0);
  param_layout->addWidget(others_group, 0, 1);

  // message parameter
  auto msg_label = new QLabel(tr("msg :"));
  msg_ = new QLineEdit;
  msg_->setPlaceholderText("message you want to write");
  auto msg_layout = new QHBoxLayout();
  msg_layout->addWidget(msg_label);
  msg_layout->addWidget(msg_);

  // checkbox
  check_z_label_ = new QLabel(tr(": Not Initialized"));
  check_z_ = new QCheckBox("get z from /points_map");
  auto check_z_layout = new QHBoxLayout();
  check_z_layout->addWidget(check_z_);
  check_z_layout->addWidget(check_z_label_);

  // button layout
  edit_button_ = new QPushButton("EDIT");
  edit_button_->setCheckable(true);

  reset_button_ = new QPushButton("RESET");
  save_button_ = new QPushButton("SAVE");
  load_button_ = new QPushButton("LOAD");
  test_button_ = new QPushButton("TEST");
  test_button_->setCheckable(true);

  connect(edit_button_, &QPushButton::clicked, this, &FakeObstacleMaker::edit);
  connect(load_button_, &QPushButton::clicked, this, &FakeObstacleMaker::load);
  connect(reset_button_, &QPushButton::clicked, this, &FakeObstacleMaker::reset);
  connect(save_button_, &QPushButton::clicked, this, &FakeObstacleMaker::save);
  connect(test_button_, &QPushButton::clicked, this, &FakeObstacleMaker::test);
  connect(check_z_, &QCheckBox::stateChanged, this, &FakeObstacleMaker::check);

  auto button_layout = new QHBoxLayout();
  button_layout->addWidget(edit_button_);
  button_layout->addWidget(load_button_);
  button_layout->addWidget(reset_button_);
  button_layout->addWidget(save_button_);
  button_layout->addWidget(test_button_);

  auto obj_info_layout = new QVBoxLayout();
  obj_info_layout->addLayout(param_layout);
  obj_info_layout->addLayout(msg_layout);
  obj_info_layout->addLayout(check_z_layout);

  obj_info_group_ = new QGroupBox(tr("Obstacle Information"));
  obj_info_group_->setLayout(obj_info_layout);
  obj_info_group_->setEnabled(false);

  //log_view_ = new QPlainTextEdit();
  //log_view_->setReadOnly(true);


  // integrate layout
  auto vlayout = new QVBoxLayout();
  vlayout->addLayout(tf_layout);
  vlayout->addWidget(obj_info_group_);
  vlayout->addLayout(button_layout);
  //vlayout->addWidget(log_view_);
  setLayout(vlayout);

  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &FakeObstacleMaker::update);
  timer->start(1000 / 30);
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::FakeObstacleMaker, rviz::Panel)
