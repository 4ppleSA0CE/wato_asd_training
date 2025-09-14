#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <cmath>


namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeMap(double resolution, double width, double height, double origin_x, double origin_y);

    void processCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    
    bool processOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);
    
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;
    
    bool shouldUpdateMap() const;
    void resetUpdateFlag();
    void setDistanceThreshold(double threshold);

  private:
    rclcpp::Logger logger_;
    
    nav_msgs::msg::OccupancyGrid global_map_;
    
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    
    double last_x_, last_y_;
    double current_x_, current_y_;
    double distance_threshold_;
    
    bool costmap_updated_;
    bool should_update_map_;
    
    double map_resolution_;
    double map_width_, map_height_;
    double map_origin_x_, map_origin_y_;
    double calculateDistance(double x1, double y1, double x2, double y2) const;
    void integrateCostmapIntoGlobalMap();
    int8_t mergeCellValues(int8_t global_value, int8_t costmap_value) const;
    bool isValidCell(int x, int y, int width, int height) const;
};

}  

#endif  
