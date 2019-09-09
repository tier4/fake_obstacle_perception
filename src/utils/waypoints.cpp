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

#include "waypoints.hpp"
#include <fstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "../library/planning_utils.h"

namespace rviz_plugins
{

void Waypoints::publishDeleteMarker()
{
  //ROS_INFO_STREAM(__FUNCTION__);
  // lambda
  auto MarkerDEL = [this](const std::string &ns, int32_t id) -> visualization_msgs::Marker{
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;
    return marker;
  };

  //ROS_INFO("Now, there are %d raw waypoints", (int32_t)wps_raw_.size());
  if(!wps_raw_.empty() && !wps_intp_.empty())
  {
    visualization_msgs::MarkerArray ma;
    for (int32_t i = 0; i < (int32_t)wps_raw_.size(); i++)
    {
      ma.markers.push_back(MarkerDEL(NS_RAW, i));
      ma.markers.push_back(MarkerDEL(NS_RAW + "_text", i));
    }

    for (int32_t i = 0; i < (int32_t)wps_intp_.size(); i++)
      ma.markers.push_back(MarkerDEL(NS_INTERPOLATED, i));

    pub_.publish(ma);
  }
}

void Waypoints::publishMarker()
{
  //ROS_INFO_STREAM(__FUNCTION__);
  // lambda
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
    marker.scale.z = 1.0;
    marker.color = Color(1.0, 1.0, 1.0, 1.0);
    marker.text = text;
    return marker;
  };


  //ROS_INFO("Now, there are %d raw waypoints", (int32_t)wps_raw_.size());
  if(!wps_raw_.empty() && !wps_intp_.empty())
  {
    visualization_msgs::MarkerArray ma;
    for (int32_t i = 0; i < (int32_t)wps_raw_.size(); i++)
    {
      auto &e = wps_raw_.at(i);
      ma.markers.push_back(ArrowADD(e, Color(0, 1.0, 0, 1.0), Scale(1.5, 0.3, 0.3), NS_RAW, i));
      ma.markers.push_back(TextADD(e, NS_RAW + "_text", i, std::to_string(i)));
    }

    for (int32_t i = 0; i < (int32_t)wps_intp_.size(); i++)
    {
      auto &e = wps_intp_.at(i);
      ma.markers.push_back(ArrowADD(e, Color(1.0, 0, 0, 1.0), Scale(1.0, 0.1, 0.1), NS_INTERPOLATED, i));
    }

    pub_.publish(ma);
  }
}

void Waypoints::add(const geometry_msgs::PointStamped& ps)
{
  //ROS_INFO_STREAM(__FUNCTION__);
  geometry_msgs::Pose pose;
  pose.position = ps.point;
  wps_raw_.push_back(pose);

  // angle interpolation
  angleInterpolation();

  wps_intp_ = planning_utils::splineInterpolatePosesWithConstantDistance(wps_raw_, 1.0);

  /*for(const auto &e : wps_raw_)
  {
    ROS_INFO("(px, py, pz, ox, oy, oz, ow): (%lf, %lf, %lf, %lf, %lf, %lf, %lf)", e.position.x, e.position.y,
             e.position.z, e.orientation.x, e.orientation.y, e.orientation.z, e.orientation.w);
  }*/

  publishMarker();
  selected_ = wps_raw_.end();
}

void Waypoints::erase(const geometry_msgs::PointStamped &ps)
{
  //ROS_INFO_STREAM(__FUNCTION__);
  publishDeleteMarker();

  auto remove_itr = std::remove_if(wps_raw_.begin(), wps_raw_.end(),
      [&ps](const geometry_msgs::Pose & pose){
    return planning_utils::calcDistance2D(pose.position, ps.point) < 1.0;
  });
  wps_raw_.erase(remove_itr, wps_raw_.end());
  wps_intp_ = planning_utils::splineInterpolatePosesWithConstantDistance(wps_raw_, 1.0);

  // angle reinterpolation
  angleInterpolation();

  publishMarker();
  selected_ = wps_raw_.end();
}

void Waypoints::select(const geometry_msgs::PointStamped &ps)
{
  //ROS_INFO_STREAM(__FUNCTION__);
  selected_ = std::find_if(wps_raw_.begin(), wps_raw_.end(),
      [&ps](const geometry_msgs::Pose & pose){
    return planning_utils::calcDistance2D(pose.position, ps.point) < 1.0;
  });
}

void Waypoints::move(const geometry_msgs::PointStamped &ps)
{
  //ROS_INFO_STREAM(__FUNCTION__);
  if(selected_ != wps_raw_.end())
  {
    publishDeleteMarker();
    selected_->position = ps.point;
    angleInterpolation();
    wps_intp_ = planning_utils::splineInterpolatePosesWithConstantDistance(wps_raw_, 1.0);
    publishMarker();
  }
  else
  {
    ROS_WARN("waypoint is not selected");
  }

}

void Waypoints::release()
{
  //ROS_INFO_STREAM(__FUNCTION__);
  selected_ = wps_raw_.end();
}

void Waypoints::reset()
{
  publishDeleteMarker();
  wps_raw_.clear();
  wps_raw_.shrink_to_fit();
  wps_intp_.clear();
  wps_intp_.shrink_to_fit();
  selected_ = wps_raw_.end();
}

void Waypoints::load(const std::vector<geometry_msgs::Pose> &wps_intp, const std::vector<geometry_msgs::Pose> &wps_raw){
  publishDeleteMarker();
  wps_raw_ = wps_raw;
  wps_intp_ = wps_intp;
  selected_ = wps_raw_.end();
  publishMarker();
}

void Waypoints::angleInterpolation()
{
  //ROS_INFO_STREAM(__FUNCTION__);
  if(wps_raw_.size() > 1)
  {
    for (int32_t i = 0; i < (int32_t)wps_raw_.size(); i++)
    {
      if(i != (int32_t)(wps_raw_.size() - 1))
      {
        auto &e = wps_raw_.at(i);
        const auto &e_next = wps_raw_.at(i + 1);
        const double x_diff = e_next.position.x - e.position.x;
        const double y_diff = e_next.position.y - e.position.y;
        double yaw = atan2(y_diff, x_diff);
        tf2::Quaternion tf2_q;
        tf2_q.setRPY(0.0, 0.0, yaw);
        e.orientation = tf2::toMsg(tf2_q);
      }
      else
      {
        wps_raw_.back().orientation = wps_raw_.at(wps_raw_.size() - 2).orientation;
      }
    }
  }
  else if(wps_raw_.size() == 1)
  {
    wps_raw_.front().orientation = geometry_msgs::Quaternion();
  }
}

}
