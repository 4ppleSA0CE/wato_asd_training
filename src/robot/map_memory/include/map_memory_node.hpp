#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <memory>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Core functionality
    robot::MapMemoryCore map_memory_;
    
    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ROS2 publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    
    // ROS2 timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double distance_threshold_;
    double update_frequency_;
    double map_resolution_;
    double map_width_;
    double map_height_;
    double map_origin_x_;
    double map_origin_y_;
    std::string costmap_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    
    // Callback functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    
    // Helper functions
    void declareParameters();
    void loadParameters();
    void initializeMap();
};

#endif 
