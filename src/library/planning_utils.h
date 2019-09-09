/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>


// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

// User defined includes
#include "interpolate.h"

// C++ includes
#include <memory>

namespace planning_utils
{

constexpr double ERROR = 1e-6;

double calcCurvature(const geometry_msgs::Point &target, const geometry_msgs::Pose &curr_pose);
double calcDistance2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q);
double calcDistSquared2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q);
double calcStopDistanceWithConstantJerk(const double &v_init, const double &j);
double calcLateralError2D(const geometry_msgs::Point &a_start, const geometry_msgs::Point &a_end,
                          const geometry_msgs::Point &b);
double calcRadius(const geometry_msgs::Point &target, const geometry_msgs::Pose &current_pose);
double convertCurvatureToSteeringAngle(double wheel_base, double kappa);
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(const std::vector<geometry_msgs::Pose> &curr_ps,
                                                      const geometry_msgs::Pose &curr_pose,
                                                      double dist_thr = 3.0,
                                                      double angle_thr = M_PI_2); // TODO: more test
bool isDirectionForward(const std::vector<geometry_msgs::Pose> &poses);

// refer from apache's pointinpoly in http://www.visibone.com/inpoly/
template <typename T>
bool isInPolygon(const std::vector<T> &polygon, const T &point);
template <>
bool isInPolygon(const std::vector<geometry_msgs::Point> &polygon, const geometry_msgs::Point &point);
double kmph2mps(double velocity_kmph);
double normalizeEulerAngle(double euler);
geometry_msgs::Point transformToAbsoluteCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);
geometry_msgs::Point transformToAbsoluteCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin); // TODO: test
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);
geometry_msgs::Point transformToRelativeCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose); // TODO: test
std::vector<geometry_msgs::Pose> splineInterpolatePosesWithConstantDistance(const std::vector<geometry_msgs::Pose> &in_poses, const double &interval_length);
geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw);
}  // namespace planning_utils
