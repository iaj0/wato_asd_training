#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <queue>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
    : Node("costmap"), 
      costmap_(robot::CostmapCore(this->get_logger())),
      resolution_(0.1), 
      i_max(300), 
      j_max(300),
      default_value_(0), 
      max_cost(100), 
      inflation_radius(1.0),
      origin_x(-15), 
      origin_y(-15) 
{
    // Initialize the occupancy grid
    occupancy_grid = std::vector<std::vector<int>>(i_max, std::vector<int>(j_max, default_value_));
    
    geometry_msgs::msg::Pose origin;
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
    double world_x = range * cos(angle);  // Convert to world coordinates
    double world_y = range * sin(angle);

    // Apply map origin offset to grid conversion
    x_grid = static_cast<int>((world_x - origin_x) / resolution_);
    y_grid = j_max - 1 - static_cast<int>((world_y - origin_y) / resolution_);
    //y_grid = static_cast<int>((world_y - origin_y) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < i_max && y_grid >= 0 && y_grid < j_max) {
        //occupancy_grid[y_grid][x_grid] = max_cost;
        costmap_data[y_grid * i_max + x_grid] = max_cost;
    }
}

void CostmapNode::inflateCostmap() {
        auto costmap_msg = nav_msgs::msg::OccupancyGrid();

    for (int y = 0; y < i_max; ++y) {
        for (int x = 0; x < j_max; ++x) {
            if (costmap_msg.data[y*i_max + x] = 100) { //occupancy_grid[y][x] == max_cost old parameter
                inflateCell(x, y);
            }
        }
    }
}

void CostmapNode::inflateCell(int obstacle_x, int obstacle_y) {
    //init costmap???
    auto costmap_msg = nav_msgs::msg::OccupancyGrid();
    costmap_msg.info.resolution = resolution_;
    costmap_msg.info.width = i_max;
    costmap_msg.info.height = j_max;
    costmap_msg.info.origin.position.x = -15;
    costmap_msg.info.origin.position.y = -15;
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;
    costmap_msg.data.resize(i_max * j_max);
    
    
    int cells_in_radius = static_cast<int>(inflation_radius / resolution_);
    std::queue<std::pair<int, int>> queue_;
    queue_.emplace(obstacle_x, obstacle_y);

    std::vector<std::vector<bool>> visited(costmap_msg.info.width, std::vector<bool>(costmap_msg.info.height, false));
    visited[obstacle_x][obstacle_y] = true;

    while (!queue_.empty()) {
        auto[x,y] = queue_.front();
        queue_.pop();
    

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if(dx == 0 && dy == 0){
                    continue;
                }
                if (costmap_msg.data[y * costmap_msg.info.width + x] == max_cost){
                    continue;
                }

                int nx = obstacle_x + dx;
                int ny = obstacle_y + dy;

                // Check if the cell is within the map bounds
                if (nx >= 0 && nx < static_cast<int>(costmap_msg.info.width) && ny >= 0 && ny < static_cast<int>(costmap_msg.info.height) && !visited[nx][ny]) {
                //calculate distance to original obstacle cell
                    double distance = std::sqrt(std::pow(nx - obstacle_x, 2) + std::pow(ny - obstacle_y, 2)) * resolution_;
                    //If within inflation radius, mark as inflated and add to queue
                    if (distance <= inflation_radius){
                        int index = ny * costmap_msg.info.width + nx;
                        if(costmap_msg.data[index] < (1- distance/inflation_radius) * max_cost){
                            costmap_msg.data[index] = (1-distance/inflation_radius) * max_cost;
                        }
                        queue_.emplace(nx, ny);
                    }
                    visited[nx][ny] = true;
                }
            }
        }
    }
}

void CostmapNode::publishCostmap() {
    auto costmap_msg = nav_msgs::msg::OccupancyGrid();

    costmap_msg.header.stamp = this->now();
    costmap_msg.header.frame_id = "sim_world";
    //costmap_msg.header = msg->header;
    costmap_msg.info.resolution = resolution_;
    costmap_msg.info.width = i_max;
    costmap_msg.info.height = j_max;
    costmap_msg.info.origin.position.x = -15;
    costmap_msg.info.origin.position.y = -15;
    costmap_msg.info.origin.position.z = 0.0;
    costmap_msg.info.origin.orientation.w = 1.0;
    
    costmap_msg.data.resize(i_max * j_max);
    for (int y = 0; y < j_max; ++y) {
        for (int x = 0; x < i_max; ++x) {
            costmap_msg.data[y * i_max + x] = occupancy_grid[y][x];
        }
    }


    //old way
    // costmap_msg.data.reserve(i_max * j_max);
    // for (const auto &row : occupancy_grid) {
    //     for (const auto &cell : row) {
    //         costmap_msg.data.push_back(cell);
    //     }
    // }

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
    RCLCPP_INFO(this->get_logger(), "Processing LaserScan with %lu points", msg->ranges.size());
    
    double angle = msg->angle_min;

    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
        double range = msg->ranges[i];

        if (range >= msg->range_min && range <= msg->range_max) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);

            RCLCPP_DEBUG(this->get_logger(), "Marking obstacle at grid (%d, %d)", x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateCostmap();
    publishCostmap();
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
  return costmap_data_;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
