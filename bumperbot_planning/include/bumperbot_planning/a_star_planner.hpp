#ifndef DIJKSTRA_PLANNER_HPP
#define DIJKSTRA_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "graph_node.hpp"

namespace bumperbot_planning
{
class AStarPlanner : public rclcpp::Node 
{
  public:
    AStarPlanner();
  private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visited_map_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;
    std::vector<int> g_cost_grid_;
    std::vector<int> parent_idx_grid_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);

    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose& pose);

    bool inBound(const GraphNode & node);

    unsigned int idx (const GraphNode & node);

    bool isFree( const GraphNode & node);


};
}// namespace bumperbot_planning

#endif // DIJKSTRA_PLANNER_HPP