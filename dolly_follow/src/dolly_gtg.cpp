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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
            this->print_once_ = true;
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

    timer_ = this->create_wall_timer(100ms, std::bind(&GoToGoal::OnTimer, this), cb_grp3_);
  }

private:
  void OnTimer()
  {
    // https://github.com/merose/diff_drive/blob/master/src/diff_drive/goal_controller.py
    
    // Skip until we have first messages
    if (!initial_goal_ || !initial_gt_) return;

    // Populate command message, all weights have been calculated by trial and error
    auto cmd_msg = std::make_shared<geometry_msgs::msg::Twist>();
    
    if (atGoal(gt_msg_, goal_pose_)) {
      // Stopping robot -- sending empty velocity message
      cmd_pub_->publish(cmd_msg);
      if (print_once_)
      {
        print_once_ = false;
        RCLCPP_INFO(this->get_logger(), "Goal reached");
        std::flush(std::cout);
      }
      return;
    }
    
    const double dX(goal_pose_.position.x - gt_msg_.position.x);
    const double dY(goal_pose_.position.y - gt_msg_.position.y);
    const double goal_heading = std::atan2(dY, dX);
    double a = goal_heading - yawFromQuaternion(gt_msg_.orientation);

    const double curr_theta = yawFromQuaternion(gt_msg_.orientation);
    const double goal_theta = yawFromQuaternion(goal_pose_.orientation); 
    const double theta = normalizePi(curr_theta - goal_theta);
    double b = -theta -a;

    const double d = getGoalDistance(gt_msg_.position, goal_pose_.position);
    const double direction = 1;
    a = normalizePi(a);
    b = normalizePi(b);

    if (std::abs(d) < lin_tol_)
    {
      cmd_msg->linear.x  = 0;
      cmd_msg->angular.z = kB * theta;
    }
    else
    {
      cmd_msg->linear.x  = kP * d * direction;
      cmd_msg->angular.z = kA * a + kB * b;
    }

    adjustVelocities(cmd_msg);

    cmd_pub_->publish(cmd_msg);
  }

  double getGoalDistance(
      const geometry_msgs::msg::Point &curr, 
      const geometry_msgs::msg::Point &goal)
  {
    const double dX(curr.x - goal.x);
    const double dY(curr.y - goal.y);

    return std::sqrt(dX * dX + dY * dY);
  }

  bool atGoal(
      const geometry_msgs::msg::Pose &curr, 
      const geometry_msgs::msg::Pose &goal)
  {
    // Linear distance from goal
    const double dist = getGoalDistance(curr.position, goal.position);
    // Angular distance from goal
    const double curr_theta = yawFromQuaternion(curr.orientation);
    const double goal_theta = yawFromQuaternion(goal.orientation);
    const double dAngle = std::abs(normalizePi(curr_theta - goal_theta));
    
    return dist < lin_tol_ && dAngle < angle_tol_;
  }

  double normalizePi(const double &angle)
  {
    double normAngle(angle);
    while (normAngle >  M_PI){
      normAngle -= TWO_M_PI_;
    }
    while (normAngle < -M_PI){
      normAngle += TWO_M_PI_;
    }
    return normAngle;
  }

  double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
  {
    tf2::Quaternion q_tf(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(q_tf);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void adjustVelocities(geometry_msgs::msg::Twist::SharedPtr &cmd)
  {
    if (cmd->linear.x > max_lin_vel_)
    {
      const double ratio = max_lin_vel_ / std::abs(cmd->linear.x);
      cmd->linear.x *= ratio;
      cmd->angular.z *= ratio;
    }
    if (cmd->angular.z > max_ang_vel_)
    {
      const double ratio = max_ang_vel_ / std::abs(cmd->angular.z);
      cmd->linear.x *= ratio;
      cmd->angular.z *= ratio;
    }
    if (cmd->linear.x > 0 && std::abs(cmd->linear.x) < min_lin_vel_)
    {
      const double ratio = min_lin_vel_ / std::abs(cmd->linear.x);
      cmd->linear.x *= ratio;
      cmd->angular.z *= ratio;
    }
    else if (cmd->linear.x == 0 && std::abs(cmd->angular.z) < min_ang_vel_)
    {
      const double ratio = min_ang_vel_ / std::abs(cmd->angular.z);
      cmd->linear.x *= ratio;
      cmd->angular.z *= ratio;
    }
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
  const double max_ang_vel_ = 0.5;
  const double min_lin_vel_ = 0.05;
  const double min_ang_vel_ = 0.1;
  const double angle_tol_ = 5 * M_PI / 180.; // 5 degrees
  const double lin_tol_ = 0.025; // 2.5 cm
  
  const double TWO_M_PI_ = 2 * M_PI;

  const double kP = 3;
  const double kA = 8;
  const double kB = -1.5;

  bool initial_goal_ = false;
  bool initial_gt_ = false;
  bool print_once_ = false;
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
  executor.spin();

  // Clean up
  rclcpp::shutdown();
  return 0;
}

