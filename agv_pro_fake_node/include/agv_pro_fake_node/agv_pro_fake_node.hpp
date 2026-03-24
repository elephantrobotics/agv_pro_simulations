#ifndef AGV_PRO_FAKE_NODE__AGV_PRO_FAKE_NODE_HPP_
#define AGV_PRO_FAKE_NODE__AGV_PRO_FAKE_NODE_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#define LEFT 0
#define RIGHT 1

class AgvProFake : public rclcpp::Node
{
public:
  AgvProFake();
  ~AgvProFake();

private:
  // ROS time
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // ROS topic publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;


  nav_msgs::msg::Odometry odom_;
  sensor_msgs::msg::JointState joint_states_;

  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;
  double last_position_[2];
  double last_velocity_[2];
  float odom_pose_[3];
  float odom_vel_[3];

  double wheel_seperation_;
  double wheel_radius_;

  // Function prototypes
  void init_parameters();
  void init_variables();
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void update_callback();
  bool update_odometry(const rclcpp::Duration & diff_time);
  void update_joint_state();
  void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);
};
#endif  // AGV_PRO_FAKE_NODE__AGV_PRO_FAKE_NODE_HPP_
