#include "costmap_core.hpp"
#include <algorithm>
#include <cstdint>

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

    void CostmapCore::initialize(int width, int height, double resolution, double origin_x, double origin_y, int8_t default_value, std::string frame_id, int max_cost) {
        width_ = width;
        height_ = height;
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        default_value_ = default_value;
        frame_id_ = frame_id;
        max_cost_ = max_cost;
        costmap_.resize(width_ * height_, default_value_);
    }
    void CostmapCore::update_costmap(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
        // Clear the costmap first
        std::fill(costmap_.begin(), costmap_.end(), default_value_);
        
        for (size_t i = 0; i < laser_scan->ranges.size(); ++i) {
            double angle = laser_scan->angle_min + i * laser_scan->angle_increment;
            double range = laser_scan->ranges[i];
            if (range < laser_scan->range_max && range > laser_scan->range_min) {
                // Calculate grid coordinates
                int x_grid, y_grid;
                convertToGrid(range, angle, x_grid, y_grid);
                if (isValidCoordinate(x_grid, y_grid)) {
                    markObstacle(x_grid, y_grid);
                }
            }
        }
        
        // Apply obstacle inflation
        inflateObstacles();
    }
    void CostmapCore::markObstacle(int x_grid, int y_grid) {
        costmap_[x_grid * height_ + y_grid] = max_cost_;
    }
    void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
        x_grid = static_cast<int>((range * std::cos(angle) + origin_x_) / resolution_);
        y_grid = static_cast<int>((range * std::sin(angle) + origin_y_) / resolution_);
    }
    nav_msgs::msg::OccupancyGrid CostmapCore::get_costmap() const {
        nav_msgs::msg::OccupancyGrid grid_msg;
        
        // Set header
        grid_msg.header.stamp = rclcpp::Clock().now();
        grid_msg.header.frame_id = frame_id_;
        
        // Set map metadata
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = width_;
        grid_msg.info.height = height_;
        grid_msg.info.origin.position.x = origin_x_;
        grid_msg.info.origin.position.y = origin_y_;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        
        // Copy grid data
        grid_msg.data = costmap_;
        
        return grid_msg;
    }
    bool CostmapCore::isValidCoordinate(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
    
    void CostmapCore::inflateObstacles() {
        // Create a copy of the current costmap for inflation
        std::vector<int8_t> inflated_costmap = costmap_;
        
        // Calculate inflation radius in grid cells
        int inflation_cells = static_cast<int>(1.0 / resolution_); // 1 meter inflation
        
        // Find all obstacle cells and inflate around them
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (costmap_[y * width_ + x] == max_cost_) { // This is an obstacle
                    // Inflate around this obstacle
                    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                            int nx = x + dx;
                            int ny = y + dy;
                            
                            if (isValidCoordinate(nx, ny)) {
                                double distance = std::sqrt(dx*dx + dy*dy) * resolution_;
                                if (distance <= 1.0) { // 1 meter inflation radius
                                    int cost = static_cast<int>(max_cost_ * (1.0 - distance / 1.0));
                                    int index = ny * width_ + nx;
                                    if (cost > inflated_costmap[index]) {
                                        inflated_costmap[index] = static_cast<int8_t>(cost);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Update the costmap with inflated values
        costmap_ = inflated_costmap;
    }
}