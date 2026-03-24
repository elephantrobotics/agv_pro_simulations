cmake_minimum_required(VERSION 3.8)
project(agv_pro_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ign_ros2_control REQUIRED)  # 关键依赖
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(mecanum_drive_controller REQUIRED)  # 确保此包已安装
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rviz2 REQUIRED)

include_directories(include)

install(DIRECTORY
  launch
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

