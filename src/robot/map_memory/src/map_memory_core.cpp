#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {}
  void MapMemoryCore::initializeMap(double resolution, double width, double height, double origin_x, double origin_y) {
    map_resolution_ = resolution;
    map_width_ = width;
    map_height_ = height;
    map_origin_x_ = origin_x;
    map_origin_y_ = origin_y;
  }
  void MapMemoryCore::processCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
    latest_costmap_ = *costmap;
  }
  bool MapMemoryCore::processOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
    current_x_ = odom->pose.pose.position.x;
    current_y_ = odom->pose.pose.position.y;
    
    double distance = std::sqrt(std::pow(current_x_ - last_x_, 2) + std::pow(current_y_ - last_y_, 2));
    
    if (distance >= distance_threshold_) {
      last_x_ = current_x_;
      last_y_ = current_y_;
      should_update_map_ = true;
      return true;
    }
    return false;
  }
  nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
    return global_map_;
  }
  bool MapMemoryCore::shouldUpdateMap() const {
    return should_update_map_;
  }
  void MapMemoryCore::resetUpdateFlag() {
    should_update_map_ = false;
  }
  void MapMemoryCore::setDistanceThreshold(double threshold) {
    distance_threshold_ = threshold;
  }
  void MapMemoryCore::integrateCostmapIntoGlobalMap() {
    for (int y = 0; y < map_height_; ++y) {
      for (int x = 0; x < map_width_; ++x) {
        global_map_.data[y * map_width_ + x] = mergeCellValues(global_map_.data[y * map_width_ + x], latest_costmap_.data[y * map_width_ + x]);
      }
    }
  }
  int8_t MapMemoryCore::mergeCellValues(int8_t global_value, int8_t costmap_value) const {
    if (costmap_value == 100) {
      return 100;
    }
    return global_value;
  }
  bool MapMemoryCore::isValidCell(int x, int y, int width, int height) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }
} 
