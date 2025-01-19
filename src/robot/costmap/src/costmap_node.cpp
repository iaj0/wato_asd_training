#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
    : Node("costmap"), 
      costmap_(robot::CostmapCore(this->get_logger())),
      resolution_(0.1), 
      i_max(30), 
      j_max(30), 
      default_value_(0), 
      max_cost(1), 
      inflation_radius(1.0) 
{
    // Initialize the occupancy grid
    occupancy_grid = std::vector<std::vector<int>>(
        i_max, std::vector<int>(j_max, default_value_)
    );

    // Create publishers and subscriptions
    string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(3000), 
        std::bind(&CostmapNode::publishMessage, this)
    );

    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10, 
        std::bind(&CostmapNode::laserScanMessage, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Costmap initialized: %d x %d, resolution: %f", i_max, j_max, resolution_);
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    x_grid = static_cast<int>(range * cos(angle) / resolution_);
    y_grid = static_cast<int>(range * sin(angle) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < i_max && y_grid >= 0 && y_grid < j_max) {
        occupancy_grid[y_grid][x_grid] = max_cost;
    }
}

void CostmapNode::inflateCostmap() {
    for (int y = 0; y < i_max; ++y) {
        for (int x = 0; x < j_max; ++x) {
            if (occupancy_grid[y][x] == max_cost) {
                inflateCell(x, y);
            }
        }
    }
}

void CostmapNode::inflateCell(int obstacle_x, int obstacle_y) {
    int cells_in_radius = static_cast<int>(inflation_radius / resolution_);

    for (int dy = -cells_in_radius; dy <= cells_in_radius; ++dy) {
        for (int dx = -cells_in_radius; dx <= cells_in_radius; ++dx) {
            int nx = obstacle_x + dx;
            int ny = obstacle_y + dy;

            if (nx < 0 || nx >= i_max || ny < 0 || ny >= j_max) {
                continue;
            }

            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
            if (distance > inflation_radius) {
                continue;
            }

            int cost = static_cast<int>(max_cost * (1 - distance / inflation_radius));
            if (cost > occupancy_grid[ny][nx]) {
                occupancy_grid[ny][nx] = cost;
            }
        }
    }
}

void CostmapNode::publishCostmap() {
    auto costmap_msg = nav_msgs::msg::OccupancyGrid();

    costmap_msg.header.stamp = this->now();
    costmap_msg.header.frame_id = "map";

    costmap_msg.info.resolution = resolution_;
    costmap_msg.info.width = i_max;
    costmap_msg.info.height = j_max;
    costmap_msg.info.origin.position.x = -i_max / 2.0 * resolution_;
    costmap_msg.info.origin.position.y = -j_max / 2.0 * resolution_;
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;

    costmap_msg.data.reserve(i_max * j_max);
    for (const auto &row : occupancy_grid) {
        for (const auto &cell : row) {
            costmap_msg.data.push_back(cell);
        }
    }

    costmap_pub_->publish(costmap_msg);
    RCLCPP_INFO(this->get_logger(), "Published costmap to /costmap");
}

void CostmapNode::publishMessage() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, ROS 2!";
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //string_pub_->publish(message);
}

void CostmapNode::laserScanMessage(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received LaserScan message: Angle Min: %f, Angle Max: %f", msg->angle_min, msg->angle_max);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];

        if (range >= msg->range_min && range <= msg->range_max) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateCostmap();
    publishCostmap();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
