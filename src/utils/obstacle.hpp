
#pragma once

// ROS includes
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// User defined includes
#include "../library/planning_utils.h"
#include "../library/pure_pursuit.h"

// C++ includes
#include <chrono>
#include <memory>

namespace rviz_plugins
{

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;


class Obstacle
{
public:
  explicit Obstacle(const std::string &node_name);
  void update();
  void reset();
  void pause();
  void needVisualization(bool val, const std::string &target_frame, int32_t density);
  PointCloudXYZI createShapePCL(int32_t density = 3) const;
  visualization_msgs::MarkerArray createMarkerArray() const;
  visualization_msgs::MarkerArray deleteMarkerArray() const;

  void setScale(double width, double length, double height)
  {
    width_ = width;
    length_ = length;
    height_ = height;
  }

  void setOthers(double velocity, double intentity, double lifetime, double respawn_delay)
  {
    velocity_ = velocity;
    intensity_ = intentity;
    lifetime_ = lifetime;
    respawn_delay_ = respawn_delay;
  }

  void setMessage(const std::string &msg) { message_ = msg; }

  void setWaypoints(const std::vector<geometry_msgs::Pose> &wps)
  {
    wps_ = wps;
    current_pose_ = wps.front();
    pp.setWaypoints(wps);
  }

  void setTargetFrame(const std::string tf) { target_frame_ = tf; }

  std::string getMessage() const { return message_; }
  std::string getTargetFrame() const {return target_frame_; }

private:
  void updatePose(double period);
  bool isLifeOver() const;
  bool isNearByGoal() const;
  bool canRespawn() const;
  void publish();

  // obstacle information
  double velocity_;
  double width_;
  double length_;
  double height_;
  double intensity_;
  double lifetime_;
  double respawn_delay_;
  std::string message_;
  std::vector<geometry_msgs::Pose> wps_;
  ros::Time spawn_stamp_;
  ros::Time finish_stamp_;
  geometry_msgs::Pose current_pose_;
  planning_utils::PurePursuit pp;

  int32_t density_;
  std::string target_frame_;

  std::string node_name_;
  bool is_visualization_;
  bool is_respawn_waiting_;
  ros::Time timestamp_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;

};

}  // namespace
