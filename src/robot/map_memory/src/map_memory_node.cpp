#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));
  map_ = std::vector<int8_t>(200 * 200, -1);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_msg_ = *msg;
  if (!update_) {
    return;
  }
  map_msg_.info.origin.position.x = x_;
  map_msg_.info.origin.position.y = y_;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  new_x_ = msg->pose.pose.position.x;
  new_y_ = msg->pose.pose.position.y;
  dis_ += std::sqrt((new_x_ - x_) * (new_x_ - x_) + (new_y_ - y_) * (new_y_ - y_));
  if (dis_ > 1.5) {
    update_ = true;
    x_ = new_x_;
    y_ = new_y_;
    heading_ = msg->pose.pose.orientation;
    dis_ = 0.0; // Reset distance counter
  }
}

void MapMemoryNode::updateMap() {
  if (!update_ || map_msg_.data.empty()) {
    return;
  }
  static constexpr int width = 200;
  static constexpr int height = 200;
  static constexpr double resolution = 0.1;
  double yaw = std::atan2(2.0 * (heading_.w * heading_.z + heading_.x * heading_.y), 1.0 - 2.0 * (heading_.y * heading_.y + heading_.z * heading_.z));
  for (auto y = 0; y < static_cast<int>(map_msg_.info.height); y++) {
    for (auto x = 0; x < static_cast<int>(map_msg_.info.width); x++) {
      int index = y * map_msg_.info.width + x;
      if (index < 0 || index >= static_cast<int>(map_msg_.data.size())){
        continue;
      }
      double local_x = (x - static_cast<int>(map_msg_.info.width) / 2) * map_msg_.info.resolution;
      double local_y = (y - static_cast<int>(map_msg_.info.height) / 2) * map_msg_.info.resolution;
      
      double global_x = map_msg_.info.origin.position.x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
      double global_y = map_msg_.info.origin.position.y + local_x * std::sin(yaw) + local_y * std::cos(yaw);

      int idx_x = static_cast<int>(std::round(global_x / resolution + width / 2));
      int idx_y = static_cast<int>(std::round(global_y / resolution + height / 2));
      if (idx_x >= 0 && idx_x < width && idx_y >= 0 && idx_y < height) {
        map_[idx_y * width + idx_x] = std::max(map_[idx_y * width + idx_x], static_cast<int8_t>(map_msg_.data[index]));
      }
    }
  }
  // Create and publish the global map message
  nav_msgs::msg::OccupancyGrid global_map_msg;
  global_map_msg.header.stamp = this->now();
  global_map_msg.header.frame_id = "map";
  global_map_msg.info.resolution = resolution;
  global_map_msg.info.width = width;
  global_map_msg.info.height = height;
  global_map_msg.info.origin.position.x = -width * resolution / 2.0;
  global_map_msg.info.origin.position.y = -height * resolution / 2.0;
  global_map_msg.data = map_;

map_pub_->publish(global_map_msg);
  update_ = false;
}

void MapMemoryNode::timerCallback() {
  updateMap();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}