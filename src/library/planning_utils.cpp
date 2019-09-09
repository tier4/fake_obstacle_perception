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

#include "planning_utils.h"

namespace planning_utils
{

double calcCurvature(const geometry_msgs::Point &target, const geometry_msgs::Pose &curr_pose)
{
  constexpr double KAPPA_MAX = 1e9;
  const double radius = calcRadius(target, curr_pose);

  if (fabs(radius) > 0)
  {
    return 1 / radius;
  }
  else
  {
    return KAPPA_MAX;
  }
}

double calcDistance2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return sqrt(dx * dx + dy * dy);
}

double calcDistSquared2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return (dx * dx + dy * dy);
}

double calcStopDistanceWithConstantJerk(const double &v_init /* initial speed [m/s] */,
                                        const double &j /* jerk [m/s^3] */)
{
  const double s = std::fabs(j);
  const double v0 = std::fabs(v_init);
  const double t2 = std::sqrt(v0 / s);
  const double t1 = t2;

  const double x1 = -s * t1 * t1 * t1 / 6 + v0 * t1;
  const double v1 = -s * t1 * t1 / 2 + v0;
  const double a1 = -s * t1;
  const double stop_dist = s * t2 * t2 * t2 / 6.0 + a1 * t2 * t2 / 2.0 + v1 * t2 + x1;

  return stop_dist;
}

/* a_vec = line_e - line_s, b_vec = point - line_s
 * a_vec x b_vec = |a_vec| * |b_vec| * sin(theta)
 *               = |a_vec| * lateral_error ( because, lateral_error = |b_vec| * sin(theta) )
 *
 * lateral_error = a_vec x b_vec / |a_vec|
 *        = (a_x * b_y - a_y * b_x) / |a_vec| */
double calcLateralError2D(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e,
                          const geometry_msgs::Point &point)
{
  tf2::Vector3 a_vec((line_e.x - line_s.x), (line_e.y - line_s.y), 0.0);
  tf2::Vector3 b_vec((point.x - line_s.x), (point.y - line_s.y), 0.0);

  double lat_err = (a_vec.length() > 0) ? a_vec.cross(b_vec).z() / a_vec.length() : 0.0;
  return lat_err;
}

double calcRadius(const geometry_msgs::Point &target, const geometry_msgs::Pose &current_pose)
{
  constexpr double RADIUS_MAX = 1e9;
  const double denominator = 2 * transformToRelativeCoordinate2D(target, current_pose).y;
  const double numerator = calcDistSquared2D(target, current_pose.position);

  if (fabs(denominator) > 0)
    return numerator / denominator;
  else
    return RADIUS_MAX;
}

double convertCurvatureToSteeringAngle(double wheel_base, double kappa)
{
  return atan(wheel_base * kappa);
}


// get closest point index from current pose
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(const std::vector<geometry_msgs::Pose> &curr_ps,
                                                      const geometry_msgs::Pose &curr_pose,
                                                      double dist_thr,
                                                      double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int32_t idx_min = -1;

  for (int32_t i = 0; i < (int32_t)curr_ps.size(); ++i)
  {
    const double ds = calcDistSquared2D(curr_ps.at(i).position, curr_pose.position);
    if (ds > dist_thr * dist_thr)
      continue;

    double yaw_pose = tf2::getYaw(curr_pose.orientation);
    double yaw_ps = tf2::getYaw(curr_ps.at(i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps);
    if (fabs(yaw_diff) > angle_thr)
      continue;

    if (ds < dist_squared_min)
    {
      dist_squared_min = ds;
      idx_min = i;
    }
  }

  return (idx_min >= 0) ? std::make_pair(true, idx_min) : std::make_pair(false, idx_min);
}

bool isDirectionForward(const std::vector<geometry_msgs::Pose> &poses)
{
  geometry_msgs::Point rel_p = transformToRelativeCoordinate2D(poses.at(1).position, poses.at(0));

  bool is_forward = (rel_p.x > 0.0) ? true : false;
  return is_forward;
}

template <typename T>
bool isInPolygon(const std::vector<T> &polygon, const T &point)
{
  // polygons with fewer than 3 sides are excluded
  if(polygon.size() < 3)
    return false;

  bool in_poly = false;
  double x1, x2, y1, y2;

  uint32_t nr_poly_points = polygon.size();
  // start with the last point to make the check last point<->first point the first one
  double xold = polygon.at(nr_poly_points - 1).x();
  double yold = polygon.at(nr_poly_points - 1).y();
  for (const auto &poly_p : polygon)
  {
    double xnew = poly_p.x();
    double ynew = poly_p.y();
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ((xnew < point.x()) == (point.x() <= xold) && (point.y() - y1) * (x2 - x1) < (y2 - y1) * (point.x() - x1))
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return (in_poly);
}

template <>
bool isInPolygon(const std::vector<geometry_msgs::Point> &polygon, const geometry_msgs::Point &point)
{
  std::vector<tf2::Vector3> polygon_conv;
  for(const auto &el : polygon)
    polygon_conv.emplace_back(el.x, el.y, el.z);

  tf2::Vector3 point_conv = tf2::Vector3(point.x, point.y, point.z);

  return isInPolygon<tf2::Vector3>(polygon_conv, point_conv);
}

double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI)
  {
    res -= (2 * M_PI);
  }
  while (res < -M_PI)
  {
    res += 2 * M_PI;
  }

  return res;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (px, py) = rot * (pu, pv) + (ox, oy)
geometry_msgs::Point transformToAbsoluteCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin)
{
  // rotation
  geometry_msgs::Point rot_p;
  double yaw = tf2::getYaw(origin.orientation);
  rot_p.x = (cos(yaw) * point.x) + ((-1) * sin(yaw) * point.y);
  rot_p.y = (sin(yaw) * point.x) + (cos(yaw) * point.y);

  // translation
  geometry_msgs::Point res;
  res.x = rot_p.x + origin.position.x;
  res.y = rot_p.y + origin.position.y;
  res.z = origin.position.z;

  return res;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

std::vector<geometry_msgs::Pose> splineInterpolatePosesWithConstantDistance(
    const std::vector<geometry_msgs::Pose> &in_poses, const double &interval_length)
{
  auto splineInterpolate =
      [](const std::vector<geometry_msgs::Pose> &in_poses, const double &interval_length) {

        if (in_poses.size() < 3)
        {
          ROS_WARN("[splineInterpolatePosesWithConstantDistance] input pose vector is empty. return empty poses.");
          return in_poses;
        }
        std::vector<geometry_msgs::Pose> out_poses;

        // convert vector<pose> to vector<double>
        std::vector<double> in_pos_x_v, in_pos_y_v, in_pos_z_v, in_yaw_v;
        for (auto &pose : in_poses)
        {
          in_pos_x_v.push_back(pose.position.x);
          in_pos_y_v.push_back(pose.position.y);
          in_pos_z_v.push_back(pose.position.z);
          in_yaw_v.push_back(tf2::getYaw(pose.orientation));
        }

        // convert yaw vector monotonically increasing
        for (unsigned int i = 1; i < in_yaw_v.size(); ++i)
        {
          const double diff = in_yaw_v[i] - in_yaw_v[i - 1];
          in_yaw_v[i] = in_yaw_v[i - 1] + normalizeEulerAngle(diff);
        }

        // calculate input vector distance
        std::vector<double> in_dist_v;
        in_dist_v.push_back(0.0);
        for (unsigned int i = 0; i < in_poses.size() - 1; ++i)
        {
          const double dx = in_poses.at(i + 1).position.x - in_poses.at(i).position.x;
          const double dy = in_poses.at(i + 1).position.y - in_poses.at(i).position.y;
          const double dz = in_poses.at(i + 1).position.z - in_poses.at(i).position.z;
          in_dist_v.push_back(in_dist_v.at(i) + std::hypot(std::hypot(dx, dy), dz));
        }

        // calculate desired distance vector
        std::vector<double> out_dist_v;
        double interpolated_dist_sum = 0.0;
        while (interpolated_dist_sum < in_dist_v.back())
        {
          out_dist_v.push_back(interpolated_dist_sum);
          interpolated_dist_sum += interval_length;
        }
        out_dist_v.push_back(in_dist_v.back());

        // apply spline interpolation
        SplineInterpolate spline_interploate;
        std::vector<double> out_pos_x_v, out_pos_y_v, out_pos_z_v, out_yaw_v;
        if (!spline_interploate.interpolate(in_dist_v, in_pos_x_v, out_dist_v, out_pos_x_v) ||
            !spline_interploate.interpolate(in_dist_v, in_pos_y_v, out_dist_v, out_pos_y_v) ||
            !spline_interploate.interpolate(in_dist_v, in_pos_z_v, out_dist_v, out_pos_z_v))
        {
          ROS_ERROR("[splineInterpolatePosesWithConstantDistance] spline interpolation failed!!");
          return out_poses;
        }

        for (int i = 0; i < (int)out_dist_v.size() - 1; ++i)
        {
          int i_next = std::min(i + 1, (int)out_dist_v.size() - 1);
          const double eps = 1e-7;
          const double dx = out_pos_x_v.at(i_next) - out_pos_x_v.at(i);
          const double dy = out_pos_y_v.at(i_next) - out_pos_y_v.at(i);
          const double den = std::fabs(dx) > eps ? dx : (dx > 0.0 ? eps : -eps);
          const double yaw_tmp = std::atan2(dy, den);
          out_yaw_v.push_back(yaw_tmp);
        }
        out_yaw_v.push_back(out_yaw_v.back());

        for (unsigned int i = 0; i < out_dist_v.size(); ++i)
        {
          geometry_msgs::Pose p;
          p.position.x = out_pos_x_v.at(i);
          p.position.y = out_pos_y_v.at(i);
          p.position.z = out_pos_z_v.at(i);
          p.orientation = getQuaternionFromYaw(out_yaw_v.at(i));
          out_poses.push_back(p);
        }
        return out_poses;
      };

  std::vector<geometry_msgs::Pose> tmp = splineInterpolate(in_poses, interval_length);
  return splineInterpolate(tmp, interval_length);
}

geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _yaw);
  return tf2::toMsg(q);
}

}  // namespace planning_utils
