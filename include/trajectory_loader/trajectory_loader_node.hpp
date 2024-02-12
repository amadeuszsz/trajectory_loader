// Copyright 2024 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAJECTORY_LOADER__TRAJECTORY_LOADER_NODE_HPP_
#define TRAJECTORY_LOADER__TRAJECTORY_LOADER_NODE_HPP_

#include <memory>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "trajectory_loader/csv_loader.hpp"


namespace trajectory_loader
{
using CSVLoaderPtr = std::unique_ptr<trajectory_loader::CSVLoader>;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using Table = std::vector<std::vector<std::string>>;
using Points = std::vector<std::vector<double>>;

class TRAJECTORY_LOADER_PUBLIC TrajectoryLoaderNode : public rclcpp::Node
{
public:
  explicit TrajectoryLoaderNode(const rclcpp::NodeOptions & options);
  void on_timer();

private:
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  CSVLoaderPtr csv_loader_{nullptr};
  Trajectory trajectory_;
  Trajectory createTrajectory(const Points & points);
};
}  // namespace trajectory_loader

#endif  // TRAJECTORY_LOADER__TRAJECTORY_LOADER_NODE_HPP_
