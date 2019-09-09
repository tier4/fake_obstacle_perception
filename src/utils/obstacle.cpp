
// C++ includes
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <dirent.h>

// ROS includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_msgs/Header.h>

// User defined includes
#include "obstacle.hpp"


namespace rviz_plugins
{

Obstacle::Obstacle(const std::string &node_name)
         : spawn_stamp_(ros::Time(0))
         , finish_stamp_(ros::Time(0))
         , node_name_(node_name)
         , is_visualization_(false)
         , is_respawn_waiting_(false)
         , timestamp_(ros::Time(0))

{
  pp.setLookaheadDistance(3.0);
  pp.setUseLerp(true);
  pp.setClosestThreshold(10.0, M_PI/2);
}

void Obstacle::update()
{
  // time initialization
  if(spawn_stamp_ == ros::Time(0))
    spawn_stamp_ = ros::Time::now();

  if(timestamp_ == ros::Time(0))
    timestamp_ = ros::Time::now();

  if(!is_respawn_waiting_)
  {
    if(isLifeOver())
    {
      is_respawn_waiting_ = true;
      //ROS_INFO("life is over: %s", message_.c_str());
    }

    if(isNearByGoal())
    {
      is_respawn_waiting_ = true;
      finish_stamp_ = ros::Time::now();
      //ROS_INFO("position is near by goal: %s", message_.c_str());

    }
    updatePose((ros::Time::now() - timestamp_).toSec());

    if(is_visualization_)
      publish();
  }
  else
  {
    //ROS_INFO("respawn waiting...: %s", message_.c_str());
    if(canRespawn())
      reset();
  }

  timestamp_ = ros::Time::now();
}

void Obstacle::reset()
{
  current_pose_ = wps_.front();
  spawn_stamp_ = ros::Time(0);
  timestamp_ = ros::Time(0);
  is_respawn_waiting_ = false;
}

void Obstacle::pause()
{
  timestamp_ = ros::Time::now();
}

void Obstacle::updatePose(double period)
{
  //ROS_INFO("updatePose: %s", message_.c_str());
  //ROS_INFO("current_pose: (x, y ,theta) = (%lf, %lf, %lf)", current_pose_.position.x, current_pose_.position.y, tf2::getYaw(current_pose_.orientation));
  //ROS_INFO("period: %lf", period);
  //ROS_INFO("wps size: %d", (int32_t)wps_.size());
  //ROS_INFO("wps.front: (x, y ,theta) = (%lf, %lf, %lf)", wps_.front().position.x, wps_.front().position.y, tf2::getYaw(wps_.front().orientation));
  pp.setCurrentPose(current_pose_);
  std::pair<bool, double> res = pp.run();

  if(res.first)
  {
    double vel = velocity_ / 3.6;
    double omega = res.second * vel;
    //ROS_INFO("vel: %lf, omega: %lf", vel, omega);
    double yaw = tf2::getYaw(current_pose_.orientation);
    double delta_x = vel * cos(yaw) * period;
    double delta_y = vel * sin(yaw) * period;
    double delta_th = omega * period;
    //ROS_INFO("delta_x: %lf, delta_y: %lf, delta_th: %lf", delta_x, delta_y, delta_th);

    current_pose_.position.x += delta_x;
    current_pose_.position.y += delta_y;

    tf2::Quaternion tf2_q;
    tf2_q.setRPY(0.0, 0.0, yaw + delta_th);
    current_pose_.orientation = tf2::toMsg(tf2_q);

    auto clst_pair = planning_utils::findClosestIdxWithDistAngThr(wps_, current_pose_, 10.0, M_PI/2);
    if(clst_pair.first)
      current_pose_.position.z = wps_.at(clst_pair.second).position.z;
  }
}

PointCloudXYZI Obstacle::createShapePCL(int32_t density) const
{

  double hl = length_ / 2.0;
  double hw = width_ / 2.0;
  double hh = height_;
  double sl = length_ / (double)density;
  double sw = width_ / (double)density;
  double sh = height_ / (double)density;

  PointCloudXYZI::Ptr pc_ptr(new PointCloudXYZI());
  double x = -hl;
  while(x < hl)
  {
    double y = -hw;
    while(y < hw)
    {
      double z = 0.0;
      while(z < hh)
      {
        pcl::PointXYZI point;
        point.x = (float)x;
        point.y = (float)y;
        point.z = (float)z;
        point.intensity = (float)intensity_;
        //ROS_INFO("point(x, y, z): (%lf, %lf, %lf", point.x, point.y, point.z);
        pc_ptr->push_back(point);
        z += sh;
      }
      y += sw;
    }
    x += sl;
  }

  PointCloudXYZI::Ptr pc_out_ptr(new PointCloudXYZI());
  Eigen::Affine3d trans_eigen;
  tf2::fromMsg(current_pose_, trans_eigen);
  pcl::transformPointCloud(*pc_ptr, *pc_out_ptr, trans_eigen);

  return *pc_out_ptr;
}

visualization_msgs::MarkerArray Obstacle::createMarkerArray() const
{
  auto Color = [](double r, double g, double b, double a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  auto Scale = [](double x, double y, double z) {
    geometry_msgs::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
  };

  auto ArrowADD = [this](const geometry_msgs::Pose &pose, const std_msgs::ColorRGBA &color,
                         const geometry_msgs::Vector3 &scale, const std::string &ns, int32_t id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale = scale;
    marker.color = color;
    return marker;
  };

  auto TextADD = [this, Color](const geometry_msgs::Pose &pose, const std::string &ns, int32_t id,
                               const std::string &text) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.pose.position.z += height_;
    marker.scale.z = 1.5;
    marker.color = Color(1.0, 1.0, 1.0, 1.0);
    marker.text = text;
    return marker;
  };

  visualization_msgs::MarkerArray ma;
  if(!wps_.empty())
  {
    for (int32_t i = 0; i < (int32_t)wps_.size(); i++)
    {
      auto &e = wps_.at(i);
      ma.markers.push_back(ArrowADD(e, Color(0, 1.0, 0, 1.0), Scale(1.5, 0.3, 0.3), message_ + "_waypoints", i));
    }
    ma.markers.push_back(TextADD(current_pose_, message_ + "_text", 0, message_));
  }
  return ma;
}

visualization_msgs::MarkerArray Obstacle::deleteMarkerArray() const
{
  auto MarkerDEL = [this](const std::string &ns, int32_t id) -> visualization_msgs::Marker{
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  };

  visualization_msgs::MarkerArray ma;
  if(!wps_.empty())
  {
    for (int32_t i = 0; i < (int32_t)wps_.size(); i++)
    {
      ma.markers.push_back(MarkerDEL(message_ + "_waypoints", i));
    }
    ma.markers.push_back(MarkerDEL(message_ + "_text", 0));
  }
  return ma;
}

bool Obstacle::isLifeOver() const
{
  double diff = (ros::Time::now() - spawn_stamp_).toSec();
  //ROS_DEBUG("diff: %lf, lifetime: %lf", diff, lifetime_);
  if(lifetime_ < 0.0 || diff < lifetime_)
    return false;
  else
    return true;
}

bool Obstacle::isNearByGoal() const
{
  const double d = planning_utils::calcDistance2D(current_pose_.position, wps_.back().position);
  const double rel_px = planning_utils::transformToRelativeCoordinate2D(wps_.back().position, current_pose_).x;
  //ROS_DEBUG("d: %lf, rel_px: %lf", d, rel_px);
  if( rel_px < 0 && d < 1.0)
    return true;
  else
    return false;
}

bool Obstacle::canRespawn() const
{
  auto diff = [this](){
    if(lifetime_ < 0.0)
      return std::make_pair((ros::Time::now() - finish_stamp_).toSec(), respawn_delay_);
    else
      return std::make_pair((ros::Time::now() - spawn_stamp_).toSec(), (lifetime_ + respawn_delay_));
  }();

  //ROS_INFO("diff: %lf, respawn_delay: %lf", diff.first, diff.second);
  if(respawn_delay_ < 0.0 || diff.first > diff.second)
    return true;
  else
    return false;


}

void Obstacle::needVisualization(bool val, const std::string &target_frame = "map",
                                 int32_t density = 3)
{
  is_visualization_ = val;
  density_ = density;
  target_frame_ = target_frame;
  pub_ = nh_.advertise<PointCloudXYZI>(node_name_ + "/pointcloud", 1);
}

void Obstacle::publish()
{
  PointCloudXYZI::Ptr obst_pcs(new PointCloudXYZI());
  *obst_pcs = createShapePCL(density_);

  std_msgs::Header header;
  header.frame_id = target_frame_;
  header.stamp = ros::Time::now();
  obst_pcs->header = pcl_conversions::toPCL(header);
  pub_.publish(obst_pcs);
}

}  // namespace
