cmake_minimum_required(VERSION 3.8)
project(agv_pro_fake_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(include)

add_executable(agv_pro_fake_node src/agv_pro_fake_node.cpp)

target_link_libraries(agv_pro_fake_node
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
)

ament_target_dependencies(agv_pro_fake_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_msgs
)

install(TARGETS
  agv_pro_fake_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
