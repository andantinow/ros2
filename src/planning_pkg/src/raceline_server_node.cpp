#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class RacelineServerNode : public rclcpp::Node
{
public:
  RacelineServerNode() : Node("raceline_server_node")
  {
    // QoS: Transient Local (Latch) for static map data
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/global_path", qos);

    // Load CSV Data
    load_raceline();

    // Timer: Publish path periodically (1Hz is enough for static global path)
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&RacelineServerNode::publish_path, this));

    RCLCPP_INFO(this->get_logger(), "Raceline Server Initialized");
  }

private:
  void load_raceline()
  {
    // Get package path (assuming raceline.csv is installed to share directory)
    // NOTE: For simplicity in dev, we use hardcoded absolute path or parameter.
    // Here we assume the file is at a known location or install destination.
    // For this quick setup, we read from source dir if not installed, or rely on install rule.
    
    // *Modify this path to match your environment if needed*
    std::string file_path = "/home/misys/el_ws/src/planning_pkg/data/raceline.csv";
    std::ifstream file(file_path);

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open: %s", file_path.c_str());
      return;
    }

    std::string line;
    // Skip header
    std::getline(file, line);

    global_path_.header.frame_id = "map";

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<double> row_data;

      while (std::getline(ss, cell, ',')) {
        row_data.push_back(std::stod(cell));
      }

      // CSV Format: s, x, y, psi, kappa, vx
      if (row_data.size() >= 3) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = row_data[1]; // x
        pose.pose.position.y = row_data[2]; // y
        pose.pose.orientation.w = 1.0;      // dummy orientation
        global_path_.poses.push_back(pose);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", global_path_.poses.size());
  }

  void publish_path()
  {
    if (global_path_.poses.empty()) return;
    
    global_path_.header.stamp = this->now();
    path_pub_->publish(global_path_);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path global_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacelineServerNode>());
  rclcpp::shutdown();
  return 0;
}
