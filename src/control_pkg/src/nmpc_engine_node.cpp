#include <chrono>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "el_interfaces/msg/adaptive_vehicle_model.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class NMPCEngineNode : public rclcpp::Node
{
public:
  NMPCEngineNode() : Node("nmpc_engine_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));

    // 1. Subscribers (Inputs)
    sub_pose_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/pose", qos, std::bind(&NMPCEngineNode::pose_callback, this, _1));

    sub_model_ = this->create_subscription<el_interfaces::msg::AdaptiveVehicleModel>(
      "/adaptive_vehicle_model", qos, std::bind(&NMPCEngineNode::model_callback, this, _1));

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planning/global_path", rclcpp::QoS(1).transient_local(), 
      std::bind(&NMPCEngineNode::path_callback, this, _1));

    // 2. Publisher (Output to Simulator)
    // Note: F1TENTH sim usually listens on '/drive' or '/sim/drive'
    pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/sim/drive", qos);

    // Control Loop (Fast: 50Hz)
    timer_ = this->create_wall_timer(
      20ms, std::bind(&NMPCEngineNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "NMPC Engine Node Initialized");
  }

private:
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { current_odom_ = *msg; has_pose_ = true; }
  void model_callback(const el_interfaces::msg::AdaptiveVehicleModel::SharedPtr msg) { current_model_ = *msg; has_model_ = true; }
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = *msg; has_path_ = true; }

  void control_loop()
  {
    // Safety Check: Wait for all inputs
    if (!has_pose_ || !has_path_) {
      return;
    }

    // --- [CORE LOGIC START] ---
    // This section will be replaced by Acados/CasADi solver later.
    // Implementing basic Pure Pursuit for testing pipeline.

    // 1. Find target point on path (Lookahead)
    double lookahead_dist = 1.0; // meters
    // Adjust lookahead based on estimated friction (Adaptive feature!)
    if (has_model_ && current_model_.tire_grip_mu < 0.9) {
        lookahead_dist = 1.5; // Look further ahead on slippery track
    }

    auto target_pt = find_target_point(lookahead_dist);

    // 2. Calculate Steering (Pure Pursuit)
    double vehicle_x = current_odom_.pose.pose.position.x;
    double vehicle_y = current_odom_.pose.pose.position.y;
    
    // Transform goal to vehicle frame (simplified 2D)
    double dx = target_pt.x - vehicle_x;
    double dy = target_pt.y - vehicle_y;
    // Extract yaw from quaternion
    double qz = current_odom_.pose.pose.orientation.z;
    double qw = current_odom_.pose.pose.orientation.w;
    double yaw = 2.0 * atan2(qz, qw);

    double local_x = cos(-yaw) * dx - sin(-yaw) * dy;
    double local_y = sin(-yaw) * dx + cos(-yaw) * dy;

    double L = 0.33; // Wheelbase
    double steering_angle = 2.0 * local_y / (lookahead_dist * lookahead_dist);
    
    // 3. Create Command
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.header.stamp = this->now();
    drive_msg.drive.steering_angle = std::max(-0.4, std::min(0.4, steering_angle));
    
    // Dynamic speed profile
    if (std::abs(steering_angle) > 0.2) {
        drive_msg.drive.speed = 2.0; // Cornering speed
    } else {
        drive_msg.drive.speed = 5.0; // Straight speed
    }
    // --- [CORE LOGIC END] ---

    pub_drive_->publish(drive_msg);
  }

  struct Point { double x, y; };

  Point find_target_point(double lookahead)
  {
    // Simple search for nearest point + lookahead
    // In real impl, track index to avoid searching whole array
    // Returning dummy target for static testing if path is empty
    if (current_path_.poses.empty()) return {0.0, 0.0};
    
    // Just pick a point for now (Logic simplified for brevity)
    // Assuming the CSV we loaded earlier, let's just target the 3rd point
    // or calculate properly if needed.
    // For the mock test, we return the 3rd waypoint from our raceline.csv
    if (current_path_.poses.size() > 3) {
        return {
            current_path_.poses[3].pose.position.x,
            current_path_.poses[3].pose.position.y
        };
    }
    return {1.0, 0.0};
  }

  // State Variables
  nav_msgs::msg::Odometry current_odom_;
  el_interfaces::msg::AdaptiveVehicleModel current_model_;
  nav_msgs::msg::Path current_path_;
  
  bool has_pose_ = false;
  bool has_model_ = false;
  bool has_path_ = false;

  // Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;
  rclcpp::Subscription<el_interfaces::msg::AdaptiveVehicleModel>::SharedPtr sub_model_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NMPCEngineNode>());
  rclcpp::shutdown();
  return 0;
}
