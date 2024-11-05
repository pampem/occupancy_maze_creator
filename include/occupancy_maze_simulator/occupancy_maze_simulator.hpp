/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
#define OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_

#include <memory>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace occupancy_maze_simulator
{

class OccupancyMazeSimulator : public rclcpp::Node
{
public:
  explicit OccupancyMazeSimulator(const rclcpp::NodeOptions & options);

private:
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace occupancy_maze_simulator

#endif  // OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
