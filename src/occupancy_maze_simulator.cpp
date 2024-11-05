/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include <occupancy_maze_simulator/occupancy_maze_simulator.hpp>

namespace occupancy_maze_simulator
{

OccupancyMazeSimulator::OccupancyMazeSimulator(const rclcpp::NodeOptions & options)
: Node("occupancy_maze_simulator", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
}

}  // namespace occupancy_maze_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_maze_simulator::OccupancyMazeSimulator)
