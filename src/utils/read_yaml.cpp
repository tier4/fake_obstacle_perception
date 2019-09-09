
#include "read_yaml.hpp"


namespace rviz_plugins
{
std::pair<bool, Obstacle> parseObstacleYAML(const std::string &filepath)
{
  ROS_INFO("read: %s", filepath.c_str());
  try
  {
    YAML::Node node = YAML::LoadFile(filepath);
    // for(const auto &n : node)
    //  ROS_INFO_STREAM(n.first << ": "  << n.second);

    auto obst = Obstacle("fake_obstacle_publisher");
    obst.setScale(node["scale"]["width"].as<double>(), node["scale"]["length"].as<double>(),
                  node["scale"]["height"].as<double>());
    obst.setOthers(node["velocity"].as<double>(), node["intensity"].as<double>(), node["lifetime"].as<double>(),
                   node["respawn_delay"].as<double>());
    obst.setMessage(node["message"].as<std::string>());
    obst.setTargetFrame(node["target_frame"].as<std::string>());

    std::vector<geometry_msgs::Pose> wps_intp;
    for (const auto &w : node["waypoints"])
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
    obst.setWaypoints(wps_intp);

    return std::make_pair(true, obst);
  }
  catch (YAML::Exception &e)
  {
    ROS_ERROR("%s", e.what());
    return std::make_pair(false, Obstacle(""));
  }
  catch (std::out_of_range &e)
  {
    ROS_ERROR("%s", e.what());
    return std::make_pair(false, Obstacle(""));
  }
}

std::tuple<bool, std::string, int32_t, std::vector<std::string>> parsePresetYAML(const std::string &filepath)
{
  ROS_INFO("read: %s", filepath.c_str());
  try
  {
    YAML::Node node = YAML::LoadFile(filepath);
    // for(const auto &n : node)
    //  ROS_INFO_STREAM(n.first << ": "  << n.second);

    std::string topic_name = node["topic_name"].as<std::string>();
    int32_t density = node["density"].as<int32_t>();

    std::vector<std::string> file_list;
    for (const auto &e : node["file_list"])
      file_list.push_back(e.as<std::string>());

    return std::make_tuple(true, topic_name, density, file_list);
  }
  catch (YAML::Exception &e)
  {
    ROS_ERROR("%s", e.what());
    return std::make_tuple(false, "", -1, std::vector<std::string>());
  }
  catch (std::out_of_range &e)
  {
    ROS_ERROR("%s", e.what());
    return std::make_tuple(false, "", -1, std::vector<std::string>());
  }
}
}