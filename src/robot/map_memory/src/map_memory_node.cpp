#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode() 
    : Node("map_memory"),  
      costmap_received_(false), 
      update_distance_threshold_(0.5), 
      should_update_map_(false), 
      last_update_x_(0), 
      last_update_y_(0) {

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, 
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, 
      std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", 10);

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(3000), 
      std::bind(&MapMemoryNode::updateGlobalMap, this)
  );
  RCLCPP_INFO(this->get_logger(), "Timer created, updateGlobalMap will be called every 500ms");


  global_map_.info.resolution = 0.1;
  global_map_.info.width = 30;
  global_map_.info.height = 30;
  global_map_.info.origin.position.x = -10;
  global_map_.info.origin.position.y = -10;
  global_map_.info.origin.position.z = 0;
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
};

double MapMemoryNode::calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}


void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
  RCLCPP_INFO(this->get_logger(), "Received a costmap update.");
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  if (calculateDistance(x, y, last_update_x_, last_update_y_) > update_distance_threshold_) {
    last_update_x_ = x;
    last_update_y_ = y;
    should_update_map_ = true;
    RCLCPP_INFO(this->get_logger(), "Received an odometry update.");
  }
}


//updating the global map
void MapMemoryNode::updateGlobalMap() {

  if (!should_update_map_) {
    RCLCPP_INFO(this->get_logger(), "should_update_map_ is false, skipping update");
    return;
  }

  if (!costmap_received_) {
    RCLCPP_WARN(this->get_logger(), "No costmap data received yet!");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Updating global map...");
  double costmap_resolution = latest_costmap_.info.resolution;
  double global_resolution = global_map_.info.resolution;

  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;

  double costmap_origin_x = latest_costmap_.info.origin.position.x;
  double costmap_origin_y = latest_costmap_.info.origin.position.y;

  for (int i = 0; i < costmap_height; ++i) {
    for (int j = 0; j < costmap_width; ++j) {
      int costmap_index = i * costmap_width + j;
      int costmap_value = latest_costmap_.data[costmap_index];

      // Convert costmap cell to global map coordinates
      double world_x = costmap_origin_x + j * costmap_resolution;
      double world_y = costmap_origin_y + i * costmap_resolution;

      // Find corresponding cell in the global map
      int global_i = static_cast<int>((world_y - global_map_.info.origin.position.y) / global_resolution);
      int global_j = static_cast<int>((world_x - global_map_.info.origin.position.x) / global_resolution);

      if (global_i >= 0 && global_i < global_map_.info.height && global_j >= 0 && global_j < global_map_.info.width) {
        int global_index = global_i * global_map_.info.width + global_j;

        // Update the global map only if the new value is not unknown (-1)
        if (costmap_value != -1) {
          global_map_.data[global_index] = costmap_value;
        }
      }
    }
  }

  should_update_map_ = false;

  // Publish the updated global map
  map_pub_->publish(global_map_);
  RCLCPP_INFO(this->get_logger(), "Published global_map_ with size: %lu", global_map_.data.size());
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
