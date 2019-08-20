// Copyright 2018 Louise Poubel.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

using namespace std::chrono_literals;

class GoToGoal : public rclcpp::Node
{
public:
  /// \brief GoToGoal node, which subscribes to laser scan messages and publishes
  /// velocity commands.
  explicit GoToGoal() : Node("go_to_goal")
  {
    cb_grp1_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    cb_grp2_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    cb_grp3_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    // Subscribe to goal pose messages
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "move_base_simple/goal",
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr _msg)
        {
            this->goal_pose_ = _msg->pose;
            this->initial_goal_ = true;
        },
        rmw_qos_profile_sensor_data,
        cb_grp1_
    );
    
    // Subscribe to ground truth messages
    gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "dolly/chassis/pose", 
        [this](const nav_msgs::msg::Odometry::SharedPtr _msg)
        {
            this->gt_msg_ = _msg->pose.pose;
            this->initial_gt_ = true;
        },
        rmw_qos_profile_sensor_data, 
        cb_grp2_
    );

    // Advertise velocity commands
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel");

    timer_ = this->create_wall_timer(500ms, std::bind(&GoToGoal::OnTimer, this), cb_grp3_);
  }

private:
  void OnTimer()
  {
    // Skip until we have first messages
    if (!initial_goal_ || !initial_gt_)
        return;
    
    // Populate command message, all weights have been calculated by trial and error
    auto cmd_msg = std::make_shared<geometry_msgs::msg::Twist>();
    
    const double dX(goal_pose_.position.x - gt_msg_.position.x);
    const double dY(goal_pose_.position.y - gt_msg_.position.y);

    const double lin_vel = std::sqrt(dX * dX + dY * dY);
    cmd_msg->linear.x = lin_vel > max_lin_vel_? max_lin_vel_ : lin_vel;

    tf2::Quaternion q(
      gt_msg_.orientation.x,
      gt_msg_.orientation.y,
      gt_msg_.orientation.z,
      gt_msg_.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double angle = std::atan2(dY, dX) - yaw;

    angle = std::fmod(angle + M_PI, TWO_M_PI_);
    if (angle < 0)
        angle += TWO_M_PI_;
    angle -= M_PI;

    cmd_msg->angular.z = std::abs(angle) < angle_tol_ ? 0 : angle;

    RCLCPP_INFO(this->get_logger(), "Goal: [%s, %f]", std::to_string(goal_pose_.position.x).c_str(), goal_pose_.position.y);
    std::flush(std::cout);

    cmd_pub_->publish(cmd_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;

  /// \brief Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  /// \brief 
  geometry_msgs::msg::Pose goal_pose_;

  /// \brief
  geometry_msgs::msg::Pose gt_msg_;

  /// \brief
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp1_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp2_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp3_;

  const double max_lin_vel_ = 0.3;
  const double TWO_M_PI_ = 2 * M_PI;
  const double angle_tol_ = 5 * M_PI / 180.;

  bool initial_goal_ = false;
  bool initial_gt_ = false;
};

int main(int argc, char * argv[])
{
  // Forward command line arguments to ROS
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  // Create a node
  auto node = std::make_shared<GoToGoal>();

  executor.add_node(node);

  // Run node until it's exited
  // rclcpp::spin(node);
  executor.spin();

  // Clean up
  rclcpp::shutdown();
  return 0;
}

