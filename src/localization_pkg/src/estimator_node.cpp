#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "el_interfaces/msg/adaptive_vehicle_model.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class EstimatorNode : public rclcpp::Node
{
public:
  EstimatorNode() : Node("estimator_node")
  {
    // QoS: Keep last 10 messages, reliable
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Sub: Odometry from simulation (Ground Truth for now)
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/sim/ego_racecar/odom", qos, std::bind(&EstimatorNode::odom_callback, this, _1));

    // Sub: Lidar scan for future SLAM integration [cite: 118]
    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&EstimatorNode::scan_callback, this, _1));

    // Pub: Estimated Pose (Corrected by EKF)
    pub_estimated_pose_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/localization/pose", qos);
      
    // Pub: Adaptive Vehicle Parameters (Tire friction, etc.) [cite: 141]
    pub_vehicle_model_ = this->create_publisher<el_interfaces::msg::AdaptiveVehicleModel>(
      "/adaptive_vehicle_model", qos);

    // Loop: 100Hz estimation rate
    timer_ = this->create_wall_timer(
      10ms, std::bind(&EstimatorNode::update_estimation, this));

    RCLCPP_INFO(this->get_logger(), "Estimator Node Initialized");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    odom_received_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // TODO: Implement scan matching here
    (void)msg; 
  }

  void update_estimation()
  {
    if (!odom_received_) return;

    // 1. State Estimation
    // Currently passing simulation Odom. Real logic needs EKF fusion here.
    auto estimated_state = last_odom_; 
    estimated_state.header.stamp = this->now();
    pub_estimated_pose_->publish(estimated_state);

    // 2. Parameter Estimation [cite: 139]
    auto model_msg = el_interfaces::msg::AdaptiveVehicleModel();
    
    // Calc velocity magnitude
    double velocity = std::sqrt(
      std::pow(last_odom_.twist.twist.linear.x, 2) + 
      std::pow(last_odom_.twist.twist.linear.y, 2)
    );

    // Dynamic Tire Friction (Mock Logic): Grip drops at high speed
    if (velocity > 5.0) {
        model_msg.tire_grip_mu = 0.85; 
    } else {
        model_msg.tire_grip_mu = 1.0;
    }

    // Set Cornering Stiffness (F1TENTH Standard)
    model_msg.cornering_stiffness_front = 100.0; 
    model_msg.cornering_stiffness_rear = 120.0;

    pub_vehicle_model_->publish(model_msg);
  }

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_estimated_pose_;
  rclcpp::Publisher<el_interfaces::msg::AdaptiveVehicleModel>::SharedPtr pub_vehicle_model_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State variables
  nav_msgs::msg::Odometry last_odom_;
  bool odom_received_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
