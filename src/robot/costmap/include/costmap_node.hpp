#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
#include <vector>

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Callback for publishing a test message
    void publishMessage();

    // Callback for processing LaserScan messages
    void laserScanMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Converts range and angle to grid indices
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);

    // Marks an obstacle in the costmap
    void markObstacle(int x_grid, int y_grid);

  private:
    robot::CostmapCore costmap_;

    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // Timer for periodic actions
    rclcpp::TimerBase::SharedPtr timer_;

    // Costmap configuration
    double resolution_; // Resolution of the costmap (meters per cell)
    int i_max, j_max;   // Grid size in terms of cells
    int default_value_; // Default value for unoccupied cells
    int max_cost;       // Maximum cost for inflated cells
    double inflation_radius; // Radius for inflation in meters

    // The 2D occupancy grid representing the costmap
    std::vector<std::vector<int>> occupancy_grid;

    // Helper methods for costmap operations
    void inflateCostmap();
    void inflateCell(int obstacle_x, int obstacle_y);
    void publishCostmap();
};

#endif // COSTMAP_NODE_HPP_
