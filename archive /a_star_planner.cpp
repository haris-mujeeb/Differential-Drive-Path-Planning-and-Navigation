#include "bumperbot_planning/a_star_planner.hpp"
#include "rmw/qos_profiles.h"
#include "tf2/time.hpp"
#include <queue>
#include <vector> // Required for std::vector
#include <algorithm> // Required for std::reverse
#include <limits> // Required for std::numeric_limits

namespace bumperbot_planning
{
AStarPlanner::AStarPlanner() : Node("a_star_node")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::QoS map_qos_(10);
  map_qos_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  map_qos_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 
    map_qos_,
    std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1)
  );

  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose",
    10,
    std::bind(&AStarPlanner::goalPoseCallback, this, std::placeholders::_1)
  );

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/a_star/path",
    10
  );

  visited_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/a_star/visited_map",
    10
  );
}

void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) 
{
  map_ = map;
  if (map_ == nullptr) 
  {
    RCLCPP_ERROR(get_logger(), "No map recieved!");
    return;
  }

  visited_map_.header.frame_id = map->header.frame_id;
  visited_map_.info = map->info;
  visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);

  // Initialize g_cost_grid_ and parent_idx_grid_ when map is received
  g_cost_grid_.assign(map_->info.height * map_->info.width, std::numeric_limits<int>::max());
  parent_idx_grid_.assign(map_->info.height * map_->info.width, -1);
}

void AStarPlanner::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{
  if (map_ == nullptr) 
  {
    RCLCPP_ERROR(get_logger(), "No map recieved!");
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_base_tf;
  try
  {
    map_to_base_tf = tf_buffer_->lookupTransform(
      map_->header.frame_id, "base_footprint", tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_ERROR(get_logger(), "Could not transform from map to base_footprint");
    return;
  }

  geometry_msgs::msg::Pose map_to_base_pose;
  map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
  map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
  map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

  nav_msgs::msg::Path path = plan(map_to_base_pose, pose_msg->pose);

  if (!path.poses.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Shortest path found");
    path_pub_->publish(path);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Path found but with no poses");
  }
}

nav_msgs::msg::Path AStarPlanner::plan(const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  if (!map_) {
    RCLCPP_WARN(get_logger(), "No map found. Returning empty path");
    return nav_msgs::msg::Path();
  }

  GraphNode start_node = worldToGrid(start_pose);
  GraphNode goal_node = worldToGrid(goal_pose);

  if (!inBound(start_node)){
    RCLCPP_WARN(get_logger(), "Start node is out of bounds");
    return nav_msgs::msg::Path();
  }
  if (!inBound(goal_node)){
    RCLCPP_WARN(get_logger(), "Goal node is out of bounds");
    return nav_msgs::msg::Path();
  }
  if (!isFree(start_node)){
    RCLCPP_WARN(get_logger(), "Start node is an obstacle or unknown location");
    return nav_msgs::msg::Path();
  }
  if (!isFree(goal_node)){
    RCLCPP_WARN(get_logger(), "Goal node is an obstacle or unknown location");
    return nav_msgs::msg::Path();
  }

  // Reset g_cost_grid_ and parent_idx_grid_ for a new plan
  // If map info changes, mapCallback will re-assign. Otherwise, just reset values.
  g_cost_grid_.assign(map_->info.height * map_->info.width, std::numeric_limits<int>::max());
  parent_idx_grid_.assign(map_->info.height * map_->info.width, -1);
  visited_map_.data.assign(map_->info.height * map_->info.width, -1); // Reset visited map data for visualization

  // Priority queue stores {g_cost, linear_index}
  std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pending_nodes;

  int start_idx = idx(start_node);
  g_cost_grid_[start_idx] = 0; // Cost from start to start is 0
  pending_nodes.push(std::make_pair(0, start_idx)); // Push {g_cost, index}

  int goal_idx = idx(goal_node);
  bool goal_reached = false;
  int active_idx = -1; // To store the index of the goal node when found

  std::vector<std::pair<int, int>> explore_directions = {
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},  // Cardinal directions
    // {-1, -1}, {-1, 1}, {1, -1}, {1, 1} // Diagonal directions
  };

  while(!pending_nodes.empty() && rclcpp::ok()) {
    int current_g_cost = pending_nodes.top().first;
    int current_idx = pending_nodes.top().second;
    pending_nodes.pop();

    // If we found a shorter path to this node already, skip this outdated entry
    if (current_g_cost > g_cost_grid_[current_idx]) {
        continue;
    }
    
    // Convert linear index back to GraphNode for convenience
    GraphNode current_node(current_idx % map_->info.width, current_idx / map_->info.width);

    // Mark the current active node as processed (final shortest path found) for visualization
    visited_map_.data.at(current_idx) = 10;

    if (current_idx == goal_idx) {
      goal_reached = true;
      active_idx = current_idx; // Store goal index
      break;
    }

    for (const auto & dir : explore_directions) {
        GraphNode neighbor_node(current_node.x + dir.first, current_node.y + dir.second);
        int neighbor_idx = idx(neighbor_node);

        // Determine if the move is diagonal or orthogonal
        int move_cost = 0;
        if (dir.first != 0 && dir.second != 0) {
          move_cost = 14; // Diagonal move
        } else {
          move_cost = 10; // Orthogonal move
        }

        if (inBound(neighbor_node) && isFree(neighbor_node)) {
            // Calculate g_cost (actual cost from start to neighbor)
            int new_g_cost = g_cost_grid_[current_idx] + move_cost + map_->data[neighbor_idx]; // Cost = parent g_cost + movement_cost + cell_cost

            // If a shorter path to this neighbor is found
            if (new_g_cost < g_cost_grid_[neighbor_idx]) {
                g_cost_grid_[neighbor_idx] = new_g_cost; // Update g_cost
                parent_idx_grid_[neighbor_idx] = current_idx; // Set parent
                
                // Push to priority queue
                pending_nodes.push(std::make_pair(new_g_cost, neighbor_idx));
            }
        }
    }
  }

  visited_map_pub_->publish(visited_map_);

  nav_msgs::msg::Path path;
  path.header.frame_id = map_->header.frame_id;
  path.header.stamp = get_clock()->now();

  if (!goal_reached) {
      RCLCPP_WARN(this->get_logger(), "No path found to goal.");
      return nav_msgs::msg::Path();
  }

  // Path reconstruction
  std::vector<geometry_msgs::msg::PoseStamped> path_poses;
  int current_path_idx = active_idx; // Start from goal index

  while(current_path_idx != start_idx && current_path_idx != -1 && rclcpp::ok())
  {
    GraphNode path_node(current_path_idx % map_->info.width, current_path_idx / map_->info.width);
    geometry_msgs::msg::Pose path_pose = gridToWorld(path_node);
    geometry_msgs::msg::PoseStamped path_pose_stamped;
    path_pose_stamped.pose = path_pose;
    path_pose_stamped.header.frame_id = map_->header.frame_id;
    path_poses.push_back(path_pose_stamped);
    current_path_idx = parent_idx_grid_[current_path_idx]; // Move to parent
  }
  // Add the start node
  GraphNode start_path_node(start_idx % map_->info.width, start_idx / map_->info.width);
  geometry_msgs::msg::Pose start_path_pose = gridToWorld(start_path_node);
  geometry_msgs::msg::PoseStamped start_path_pose_stamped;
  start_path_pose_stamped.pose = start_path_pose;
  start_path_pose_stamped.header.frame_id = map_->header.frame_id;
  path_poses.push_back(start_path_pose_stamped);

  std::reverse(path_poses.begin(), path_poses.end());
  path.poses = path_poses;
  return path;
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & node)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
  pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
  return pose;
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
 int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x)/ map_->info.resolution);
 int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y)/ map_->info.resolution);
 return GraphNode(grid_x, grid_y);
}

bool AStarPlanner::inBound(const GraphNode & node)
{
  bool xInBound = (0 <= node.x) && (node.x <= static_cast<int>(map_->info.width));
  bool yInBound = (0 <= node.y) && (node.y <= static_cast<int>(map_->info.height));
  return xInBound && yInBound ;
}

unsigned int AStarPlanner::idx(const GraphNode & node)
{
  return  map_->info.width * node.y + node.x;
}

bool AStarPlanner::isFree(const GraphNode & node)
{
  return (0 <= map_->data[idx(node)]) && (map_->data[idx(node)] < 99);
}

} // namespace bumperbot_planning

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bumperbot_planning::AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}