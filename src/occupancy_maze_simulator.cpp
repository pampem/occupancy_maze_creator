/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include <occupancy_maze_simulator/occupancy_maze_simulator.hpp>

#include <random>

namespace occupancy_maze_simulator
{

OccupancyMazeSimulator::OccupancyMazeSimulator(const rclcpp::NodeOptions & options)
: Node("occupancy_maze_simulator", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  occupancy_grid_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
  goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_position", 10);

  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&OccupancyMazeSimulator::twist_callback, this, std::placeholders::_1));

  auto obstacles = generate_maze_obstacles(10, 1, {50, 50});
  publish_maze(obstacles, {50, 50}, 1.0);
}

void OccupancyMazeSimulator::publish_maze(
  const std::vector<Obstacle> & obstacles, const std::pair<int, int> & area_size, double resolution)
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.info.resolution = resolution;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.info.width = area_size.first;
  grid_msg.info.height = area_size.second;
  grid_msg.info.origin.position.x = 0.0;
  grid_msg.info.origin.position.y = 0.0;
  grid_msg.data.resize(area_size.first * area_size.second, 0);  // 非占有セルは0で初期化
  grid_msg.header.frame_id = "odom";
  grid_msg.header.stamp = this->get_clock()->now();

  // 障害物を配置
  for (const auto & obstacle : obstacles) {
    int cell_x = static_cast<int>(std::round(obstacle.x / resolution));
    int cell_y = static_cast<int>(std::round(obstacle.y / resolution));
    if (cell_x >= 0 && cell_x < area_size.first && cell_y >= 0 && cell_y < area_size.second) {
      int index = cell_y * area_size.first + cell_x;
      grid_msg.data[index] = 100;  // 障害物セルを設定
    } else {
      RCLCPP_WARN(this->get_logger(), "Obstacle out of grid bounds at (%d, %d)", cell_x, cell_y);
    }
  }
  RCLCPP_INFO(this->get_logger(), "GridMap Published ");
  occupancy_grid_publisher_->publish(grid_msg);
}

Obstacle OccupancyMazeSimulator::create_obstacle(
  double x, double y, double width, double height, double angle)
{
  return Obstacle{x, y, width, height, angle};
}

std::vector<Obstacle> OccupancyMazeSimulator::generate_random_obstacles(
  int num_obstacles, const std::pair<int, int> & area_size)
{
  std::vector<Obstacle> obstacles;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist_x(6, area_size.first - 6);
  std::uniform_real_distribution<> dist_y(6, area_size.second - 6);
  std::uniform_real_distribution<> dist_width(5, 10);
  std::uniform_real_distribution<> dist_height(5, 10);
  std::uniform_real_distribution<> dist_angle(0, 180);

  obstacles.reserve(num_obstacles);
  for (int i = 0; i < num_obstacles; ++i) {
    obstacles.push_back(create_obstacle(
      dist_x(gen), dist_y(gen), dist_width(gen), dist_height(gen), dist_angle(gen)));
  }
  return obstacles;
}

std::vector<Obstacle> OccupancyMazeSimulator::generate_maze_obstacles(
  int /*grid_size*/, int cell_size, const std::pair<int, int> & area_size)
{
  std::vector<Obstacle> obstacles;
  int num_cells_x = area_size.first / cell_size;
  int num_cells_y = area_size.second / cell_size;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dist(0, 3);  // 0から3の乱数生成

  for (int i = 0; i < num_cells_x; ++i) {
    for (int j = 0; j < num_cells_y; ++j) {
      if (
        (i * cell_size <= 10 && j * cell_size <= 10) ||
        (area_size.first - 10 <= i * cell_size && area_size.second - 10 <= j * cell_size)) {
        continue;
      }

      if (dist(gen) == 0) {  // ランダムに障害物を配置
        double x = i * cell_size + static_cast<double>(cell_size) / 2.0;
        double y = j * cell_size + static_cast<double>(cell_size) / 2.0;
        double width = (dist(gen) % 2 == 0) ? cell_size : 2 + (dist(gen) % 2);
        double height = (width == cell_size) ? 2 + (dist(gen) % 2) : cell_size;
        obstacles.push_back(create_obstacle(x, y, width, height, 0));
      }
    }
  }
  return obstacles;
}

void OccupancyMazeSimulator::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received Twist message: linear=%f, angular=%f", msg->linear.x,
    msg->angular.z);
}

}  // namespace occupancy_maze_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_maze_simulator::OccupancyMazeSimulator)
