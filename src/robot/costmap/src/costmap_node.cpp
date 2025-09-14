#include <chrono>
#include <memory>
#include <string>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  this->declare_parameter("width", 100);
  this->declare_parameter("height", 100);
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("origin_x", -5.0);
  this->declare_parameter("origin_y", -5.0);
  this->declare_parameter("default_value", 0);
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("max_cost", 100);

  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  resolution_ = this->get_parameter("resolution").as_double();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  default_value_ = this->get_parameter("default_value").as_int();
  frame_id_ = this->get_parameter("frame_id").as_string();
  max_cost_ = this->get_parameter("max_cost").as_int();

  costmap_.initialize(width_, height_, resolution_, origin_x_, origin_y_, default_value_, frame_id_, max_cost_);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  RCLCPP_INFO(this->get_logger(), "Costmap initialized");
}
 
void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.update_costmap(msg);
  auto costmap = costmap_.get_costmap();
  costmap_pub_->publish(costmap);
  RCLCPP_INFO(this->get_logger(), "LaserScan received");
}
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}