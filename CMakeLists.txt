cmake_minimum_required(VERSION 2.8.3)
project(fake_obstacle_perception)
add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rviz
        geometry_msgs
        event_capture
        tf2
        tf2_ros
        pcl_ros
        tf2_geometry_msgs
        visualization_msgs)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS roscpp rviz geometry_msgs event_capture tf2 tf2_ros pcl_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(QT_HEADER_FILES
        src/maker/widget.hpp
        src/publisher/widget.hpp
)

set(QT_SOURCE_FILES
        src/maker/widget.cpp
        src/publisher/widget.cpp
        src/utils/waypoints.cpp
        src/utils/obstacle.cpp
        src/utils/points_map.cpp
        src/utils/read_yaml.cpp
        src/library/interpolate.cpp
        src/library/pure_pursuit.cpp
        src/library/planning_utils.cpp
)

add_library(${PROJECT_NAME}
  ${QT_HEADER_FILES}
  ${QT_SOURCE_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  yaml-cpp
)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS})


add_executable(auto_publisher
        src/auto_publisher/auto_publisher_node.cpp
        src/auto_publisher/auto_publisher.cpp
        src/utils/waypoints.cpp
        src/utils/obstacle.cpp
        src/utils/read_yaml.cpp
        src/library/interpolate.cpp
        src/library/pure_pursuit.cpp
        src/library/planning_utils.cpp)
target_link_libraries(auto_publisher ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(auto_publisher ${catkin_EXPORTED_TARGETS})
