#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initialize(int width, int height, double resolution, double origin_x, double origin_y, int8_t default_value, std::string frame_id, int max_cost);

    void update_costmap(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

    nav_msgs::msg::OccupancyGrid get_costmap() const;
    
    void inflateObstacles();
    
    void markObstacle(int x_grid, int y_grid);
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    bool isValidCoordinate(int x, int y) const;

  private:
    rclcpp::Logger logger_;

    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    int8_t default_value_;
    std::string frame_id_;
    int max_cost_;
    std::vector<int8_t> costmap_;
};

}  

#endif  