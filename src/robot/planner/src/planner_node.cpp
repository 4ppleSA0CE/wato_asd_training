#include "planner_node.hpp"
#include <vector>
#include <algorithm>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  odom_sub_ = this->create_subscription<geometry_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
}

void PlannerNode::odomCallback(const geometry_msgs::msg::Odometry::SharedPtr msg) {
  pose_msg_ = msg->pose.pose;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_msg_ = *msg;
  if (plan_flag_ == PlannerState::waiting_for_robot_to_reach_goal && !map_msg_.data.empty()) {
    createPath();
  }
}
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_msg_ = *msg;
  plan_flag_ = PlannerState::waiting_for_robot_to_reach_goal;
  goal_flag_ = true;
  createPath();
}

void PlannerNode::timerCallback() {
  if (plan_flag_ == PlannerState::waiting_for_robot_to_reach_goal && goal_flag_) {
    double dis = std::sqrt(std::pow(goal_msg_.pose.position.x - pose_msg_.position.x, 2) + std::pow(goal_msg_.pose.position.y - pose_msg_.position.y, 2));
    if (dis < 0.5){
      plan_flag_ = PlannerState::waiting_for_goal;
      nav_msgs::msg::Path path;
      path_pub_->publish(path);
    }else createPath();
  }
}

CellIndex PlannerNode::getCellIndex(const geometry_msgs::msg::Pose& pose) {
  int x = static_cast<int>(pose.position.x/map_msg_.info.resolution + static_cast<int>(map_msg_.info.width/2));
  int y = static_cast<int>(pose.position.y/map_msg_.info.resolution + static_cast<int>(map_msg_.info.height/2));
  return CellIndex(x, y);
}

geometry_msgs::msg::Pose PlannerNode::getPose(const CellIndex& index) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = (index.x - static_cast<int>(map_msg_.info.width/2)) * map_msg_.info.resolution;
  pose.position.y = (index.y - static_cast<int>(map_msg_.info.height/2)) * map_msg_.info.resolution;
  pose.orientation.w = 1.0;
  pose.orientation.z = 0.0;
  return pose;
}

double PlannerNode::heuristic(const CellIndex& goal, const CellIndex& current) {
  return std::sqrt(std::pow(goal.x - current.x, 2) + std::pow(goal.y - current.y, 2));
}

bool PlannerNode::checkValid(const CellIndex& index) {
  // Bounds check for the cell itself
  if (index.x < 0 || index.x >= static_cast<int>(map_msg_.info.width) ||
      index.y < 0 || index.y >= static_cast<int>(map_msg_.info.height)) {
    return false;
  }

  // Robot footprint parameters
  const double ROBOT_LENGTH = 1.5;  // meters (behind the robot's front cell)
  const double ROBOT_WIDTH  = 1.0;  // meters (centered on robot)
  const double GRID_STEP    = std::max(0.1, static_cast<double>(map_msg_.info.resolution));
  const int OBSTACLE_THRESHOLD = 50; // occupancy threshold

  // Robot orientation (yaw)
  const double yaw = std::atan2(2.0 * (pose_msg_.orientation.w * pose_msg_.orientation.z + pose_msg_.orientation.x * pose_msg_.orientation.y), 1.0 - 2.0 * (pose_msg_.orientation.y * pose_msg_.orientation.y + pose_msg_.orientation.z * pose_msg_.orientation.z));

  // World coordinates of the given cell (treated as robot front)
  geometry_msgs::msg::Pose world_pose = getPose(index);
  const double rx = world_pose.position.x;
  const double ry = world_pose.position.y;

  // Check all points in the rectangular footprint
  for (double back = 0.0; back <= ROBOT_LENGTH; back += GRID_STEP) {
    for (double side = -ROBOT_WIDTH / 2.0; side <= ROBOT_WIDTH / 2.0; side += GRID_STEP) {
      // Point in robot frame (front at 0,0). Negative x is behind the front
      const double local_x = -back;
      const double local_y = side;

      // Transform to world frame
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double wx = rx + cos_yaw * local_x - sin_yaw * local_y;
      const double wy = ry + sin_yaw * local_x + cos_yaw * local_y;

      // Convert to grid indices
      const int map_x = static_cast<int>(wx / map_msg_.info.resolution + static_cast<int>(map_msg_.info.width) / 2);
      const int map_y = static_cast<int>(wy / map_msg_.info.resolution + static_cast<int>(map_msg_.info.height) / 2);

      // Bounds check
      if (map_x < 0 || map_x >= static_cast<int>(map_msg_.info.width) ||
          map_y < 0 || map_y >= static_cast<int>(map_msg_.info.height)) {
        return false; // out of map bounds treated as collision
      }

      const int map_idx = map_y * static_cast<int>(map_msg_.info.width) + map_x;
      if (map_msg_.data[map_idx] >= OBSTACLE_THRESHOLD) {
        return false; // collision detected
      }
    }
  }

  return true; // all footprint points are free
}

void PlannerNode::createPath() {
  if (!goal_flag_ || map_msg_.data.empty()) {
    return;
  }

  CellIndex start = getCellIndex(pose_msg_);
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.position.x = goal_msg_.point.x;
  goal_pose.position.y = goal_msg_.point.y;
  goal_pose.orientation.w = 1.0;
  CellIndex goal = getCellIndex(goal_pose);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(goal, start));

  bool found = false;
  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    if (current == goal) {
      found = true;
      break;
    }
    open_set.pop();

    std::vector<CellIndex> neighbors = {
      CellIndex(current.x + 1, current.y),
      CellIndex(current.x - 1, current.y),
      CellIndex(current.x, current.y + 1),
      CellIndex(current.x, current.y - 1)
    };

    for (const auto &neighbor : neighbors) {
      if (!checkValid(neighbor)) continue;
      double tentative_g = g_score[current] + 1.0;
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(goal, neighbor);
        open_set.emplace(neighbor, f);
      }
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "map";
  if (found) {
    std::vector<CellIndex> cells;
    CellIndex current = goal;
    while (current != start) {
      cells.push_back(current);
      current = came_from[current];
    }
    cells.push_back(start);
    std::reverse(cells.begin(), cells.end());
    for (const auto &cell : cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose = getPose(cell);
      path.poses.push_back(pose);
    }
  }
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
