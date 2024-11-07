/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
#define OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <utility>
#include <vector>

namespace occupancy_maze_simulator
{

struct Obstacle
{
  double x;
  double y;
  double width;
  double height;
  double angle;
};

class OccupancyMazeSimulator : public rclcpp::Node
{
public:
  explicit OccupancyMazeSimulator(const rclcpp::NodeOptions & options);

private:
  nav_msgs::msg::OccupancyGrid create_grid_map(
    const std::vector<Obstacle> & obstacles,
    const std::pair<int, int> & area_size, double resolution);

  void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);

  static std::vector<Obstacle> generate_random_obstacles(
    int num_obstacles, const std::pair<int, int> & area_size);

  static std::vector<Obstacle> generate_maze_obstacles(
    int grid_size, int cell_size, const std::pair<int, int> & area_size);

  static Obstacle create_obstacle(double x, double y, double width, double height, double angle);

  static bool is_path_to_goal(
    const nav_msgs::msg::OccupancyGrid & grid_map, const std::pair<int, int> & start,
    const std::pair<int, int> & goal);

  nav_msgs::msg::OccupancyGrid occupancy_grid_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
};

}  // namespace occupancy_maze_simulator

#endif  // OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
