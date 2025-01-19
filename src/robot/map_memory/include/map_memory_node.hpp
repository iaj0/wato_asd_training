#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <cmath>

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    // Callback functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Helper functions
    void updateGlobalMap();
    static double calculateDistance(double x1, double y1, double x2, double y2);

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables for map fusion and position tracking
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool should_update_map_;
    double last_update_x_;
    double last_update_y_;
    bool costmap_received_;
    double update_distance_threshold_; // e.g., 1.5 meters
};

#endif // MAP_MEMORY_NODE_HPP_
