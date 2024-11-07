/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include <occupancy_maze_simulator/occupancy_maze_simulator.hpp>

#include <queue>
#include <random>

namespace occupancy_maze_simulator
{

OccupancyMazeSimulator::OccupancyMazeSimulator(const rclcpp::NodeOptions & options)
: Node("occupancy_maze_simulator", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  this->declare_parameter<std::string>("obstacle_mode", "maze");  // 障害物配置モード:maze / random
  this->declare_parameter<float>("gridmap.resolution", 1);
  this->declare_parameter<float>("gridmap.x", 50);
  this->declare_parameter<float>("gridmap.y", 50);
  this->declare_parameter<std::vector<int>>("start_position", {1, 1});
  this->declare_parameter<std::vector<int>>("goal_position", {48, 48});

  std::string obstacle_mode = this->get_parameter("obstacle_mode").as_string();
  float resolution = this->get_parameter("gridmap.resolution").as_double();
  float gridmap_x = this->get_parameter("gridmap.x").as_double();
  float gridmap_y = this->get_parameter("gridmap.y").as_double();
  auto start_vec = this->get_parameter("start_position").as_integer_array();
  auto goal_vec = this->get_parameter("goal_position").as_integer_array();

  num_cells_x_ = static_cast<int>(gridmap_x / resolution);
  num_cells_y_ = static_cast<int>(gridmap_y / resolution);
  cell_size_ = resolution;
  if (start_vec.size() == 2) start_position_ = {start_vec[0], start_vec[1]};
  if (goal_vec.size() == 2) goal_position_ = {goal_vec[0], goal_vec[1]};

  occupancy_grid_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
  goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_position", 10);

  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&OccupancyMazeSimulator::twist_callback, this, std::placeholders::_1));

  nav_msgs::msg::OccupancyGrid grid_map;
  std::vector<Obstacle> obstacles;
  int count = 0;
  do {
    if (obstacle_mode == "maze") {
      obstacles = generate_maze_obstacles(cell_size_, {num_cells_x_, num_cells_y_});
    } else if (obstacle_mode == "random") {
      obstacles = generate_random_obstacles(10, {num_cells_x_, num_cells_y_});
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid obstacle_mode. Defaulting to maze mode.");
      obstacles = generate_maze_obstacles(cell_size_, {num_cells_x_, num_cells_y_});
    }

    grid_map = create_grid_map(obstacles, {num_cells_x_, num_cells_y_}, cell_size_);
    if (!is_path_to_goal(grid_map, start_position_, goal_position_)) {
      RCLCPP_WARN(this->get_logger(), "No path to the goal exists. Regenerating obstacles.");
      ++count;
    }
    if (count >= 100) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't create valid obstacles. Check your parameters.");
      break;
    }
  } while (!is_path_to_goal(grid_map, start_position_, goal_position_));
  if (count < 100) {
    RCLCPP_INFO(this->get_logger(), "A path to the goal exists.");
    occupancy_grid_publisher_->publish(grid_map);  // 有効なGridmapをパブリッシュ
  }
}

nav_msgs::msg::OccupancyGrid OccupancyMazeSimulator::create_grid_map(
  const std::vector<Obstacle> & obstacles, const std::pair<int, int> & area_size, float cell_size)
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.info.resolution = cell_size;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.info.width = area_size.first;
  grid_msg.info.height = area_size.second;
  grid_msg.info.origin.position.x = 0.0;
  grid_msg.info.origin.position.y = 0.0;
  grid_msg.data.resize(area_size.first * area_size.second, 0);  // 非占有セルは0で初期化
  grid_msg.header.frame_id = "odom";
  grid_msg.header.stamp = this->get_clock()->now();

  // 障害物の配置
  for (const auto & obstacle : obstacles) {
    int cell_x = static_cast<int>(std::round(obstacle.x / cell_size));
    int cell_y = static_cast<int>(std::round(obstacle.y / cell_size));
    if (cell_x >= 0 && cell_x < area_size.first && cell_y >= 0 && cell_y < area_size.second) {
      int index = cell_y * area_size.first + cell_x;
      grid_msg.data[index] = 100;  // 障害物セルを設定
    } else {
      RCLCPP_WARN(this->get_logger(), "Obstacle out of grid bounds at (%d, %d)", cell_x, cell_y);
    }
  }
  return grid_msg;
}

bool OccupancyMazeSimulator::is_path_to_goal(
  const nav_msgs::msg::OccupancyGrid & grid_map, const std::pair<int, int> & start,
  const std::pair<int, int> & goal)
{
  const int width = grid_map.info.width;
  const int height = grid_map.info.height;
  const auto & data = grid_map.data;

  std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
  std::queue<std::pair<int, int>> queue;
  queue.push(start);
  visited[start.first][start.second] = true;

  const int dx[] = {1, -1, 0, 0};
  const int dy[] = {0, 0, 1, -1};

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    if (x == goal.first && y == goal.second) {
      return true;
    }

    for (int i = 0; i < 4; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];

      if (
        nx >= 0 && nx < width && ny >= 0 && ny < height && data[ny * width + nx] == 0 &&
        !visited[nx][ny]) {
        queue.emplace(nx, ny);
        visited[nx][ny] = true;
      }
    }
  }
  return false;
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
  float cell_size, const std::pair<int, int> & area_size)
{
  std::vector<Obstacle> obstacles;
  int num_cells_x = area_size.first;
  int num_cells_y = area_size.second;

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
