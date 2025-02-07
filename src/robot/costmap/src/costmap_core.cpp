#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

// void CostmapCore::initCostmap(double resolution, int width, int height, 
//     geometry_msgs::msg::Pose origin, double inflation_radius) {
//   costmap_data_->info.resolution = resolution;
//   costmap_data_->info.width = width;
//   costmap_data_->info.height = height;
//   costmap_data_->info.origin = origin;
//   costmap_data_->data.assign(width * height, -1);

//   inflation_radius_ = inflation_radius;
//   inflation_cells_ = static_cast<int>(inflation_radius / resolution);

//   RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
// }

































}