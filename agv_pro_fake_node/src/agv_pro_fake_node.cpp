#include "agv_pro_fake_node/agv_pro_fake_node.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;

AgvProFake::AgvProFake()
: Node("agv_pro_fake_node")
{
  /************************************************************
  ** Initialise ROS parameters
  ************************************************************/
  init_parameters();

  /************************************************************
  ** Initialise variables
  ************************************************************/
  init_variables();

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
  tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);

  // Initialise subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", \
    qos, \
    std::bind(
      &AgvProFake::command_velocity_callback, \
      this, \
      std::placeholders::_1));

  /************************************************************
  ** initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&AgvProFake::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "AGV Pro fake node has been initialised");
}

AgvProFake::~AgvProFake()
{
  RCLCPP_INFO(this->get_logger(), "AGV Pro fake node has been terminated");
}

/********************************************************************************
** Init functions
********************************************************************************/
void AgvProFake::init_parameters()
{
  // Declare parameters that may be set on this node
  this->declare_parameter<std::string>("joint_states_frame");
  this->declare_parameter<std::string>("odom_frame");
  this->declare_parameter<std::string>("base_frame");
  this->declare_parameter<double>("wheels.separation");
  this->declare_parameter<double>("wheels.radius");

  // Get parameters from yaml
  this->get_parameter_or<std::string>(
    "joint_states_frame", \
    joint_states_.header.frame_id, \
    "base_footprint");
  this->get_parameter_or<std::string>("odom_frame", odom_.header.frame_id, "odom");
  this->get_parameter_or<std::string>("base_frame", odom_.child_frame_id, "base_footprint");
  this->get_parameter_or<double>("wheels.separation", wheel_seperation_, 0.3598); // 从URDF计算: 0.1799*2
  this->get_parameter_or<double>("wheels.radius", wheel_radius_, 0.05); // 需要根据实际模型调整
}

void AgvProFake::init_variables()
{
  // Initialise variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_ = 0.0;
  goal_angular_velocity_ = 0.0;
  cmd_vel_timeout_ = 1.0;
  last_position_[LEFT] = 0.0;
  last_position_[RIGHT] = 0.0;
  last_velocity_[LEFT] = 0.0;
  last_velocity_[RIGHT] = 0.0;

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  // 根据您的URDF设置关节名称
  joint_states_.name.push_back("left_front_wheel_joint");
  joint_states_.name.push_back("left_rear_wheel_joint");
  joint_states_.name.push_back("right_front_wheel_joint");
  joint_states_.name.push_back("right_rear_wheel_joint");
  joint_states_.position.resize(4, 0.0);
  joint_states_.velocity.resize(4, 0.0);
  joint_states_.effort.resize(4, 0.0);

  prev_update_time_ = this->now();
  last_cmd_vel_time_ = this->now();
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void AgvProFake::command_velocity_callback(
  const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = this->now();

  goal_linear_velocity_ = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT] = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + \
    (goal_angular_velocity_ * wheel_seperation_ / 2);
}

/********************************************************************************
** Update functions
********************************************************************************/
void AgvProFake::update_callback()
{
  rclcpp::Time time_now = this->now();
  rclcpp::Duration duration(time_now - prev_update_time_);
  prev_update_time_ = time_now;

  // zero-ing after timeout (stop the robot if no cmd_vel)
  if ((time_now - last_cmd_vel_time_).nanoseconds() / 1e9 > cmd_vel_timeout_) {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  // odom
  update_odometry(duration);
  odom_.header.stamp = time_now;
  odom_pub_->publish(odom_);

  // joint_states
  update_joint_state();
  joint_states_.header.stamp = time_now;
  joint_states_pub_->publish(joint_states_);

  // tf
  geometry_msgs::msg::TransformStamped odom_tf;
  update_tf(odom_tf);
  tf2_msgs::msg::TFMessage odom_tf_msg;
  odom_tf_msg.transforms.push_back(odom_tf);
  tf_pub_->publish(odom_tf_msg);
}

bool AgvProFake::update_odometry(const rclcpp::Duration & duration)
{
  double wheel_l, wheel_r;  // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];
  double step_time = duration.nanoseconds() / 1e9;  // [sec]

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = 0.0;

  // v = translational velocity [m/s]
  // w = rotational velocity [rad/s]
  v[LEFT] = wheel_speed_cmd_[LEFT];
  w[LEFT] = v[LEFT] / wheel_radius_;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / wheel_radius_;

  last_velocity_[LEFT] = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT] * step_time;
  wheel_r = w[RIGHT] * step_time;

  if (isnan(wheel_l)) {
    wheel_l = 0.0;
  }

  if (isnan(wheel_r)) {
    wheel_r = 0.0;
  }

  last_position_[LEFT] += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_seperation_;

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / step_time;     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / step_time;  // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, odom_pose_[2]);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}

void AgvProFake::update_joint_state()
{
  // 设置所有左轮的相同位置和速度
  joint_states_.position[0] = last_position_[LEFT];  // left_front
  joint_states_.position[1] = last_position_[LEFT];  // left_rear
  joint_states_.velocity[0] = last_velocity_[LEFT];  // left_front
  joint_states_.velocity[1] = last_velocity_[LEFT];  // left_rear
  
  // 设置所有右轮的相同位置和速度
  joint_states_.position[2] = last_position_[RIGHT];  // right_front
  joint_states_.position[3] = last_position_[RIGHT];  // right_rear
  joint_states_.velocity[2] = last_velocity_[RIGHT];  // right_front
  joint_states_.velocity[3] = last_velocity_[RIGHT];  // right_rear
}

void AgvProFake::update_tf(geometry_msgs::msg::TransformStamped & odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*****************************************************************************
** Main
*****************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvProFake>());
  rclcpp::shutdown();

  return 0;
}
