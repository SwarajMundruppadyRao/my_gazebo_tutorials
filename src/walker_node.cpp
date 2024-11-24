/**
 * @brief Implementation of the Walker Algorithm
 * @file walker_node.cpp
 * @date November 19 2024
 * @version 1.2
 * @author Swaraj Mundruppady Rao (swarajmr@umd.edu)
 * @copyright Copyright (c) 2024
 */

// Include the necessary C++ Libraries
#include <chrono>
#include <limits>
#include <memory>

// Include the necessary header files
#include "walker/walker_states.hpp"

// WalkerContext Implementation
WalkerContext::WalkerContext(rclcpp::Node::SharedPtr node)
    : node_(node),
      obstacle_detected_(false),
      rotate_clockwise_(true),
      rotation_start_time_(),
      is_rotating_(false) {
  // Initialize publisher for robot velocity commands
  cmd_vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Initialize subscription for laser scan data
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&WalkerContext::updateObstacleDetection, this,
                std::placeholders::_1));

  // Set the initial state to MoveForwardState
  current_state_ = std::make_shared<MoveForwardState>();
  clock_ = node_->get_clock();
}

// Implement the setState() method to set the current state of the walker.
void WalkerContext::setState(std::shared_ptr<WalkerState> state) {
  current_state_ = state;
}

// Implement the execute() method to execute the logic of the current state.
void WalkerContext::execute() {
  if (current_state_) {
    current_state_->execute(*this);
  }
}

// Implement the updateObstacleDetection() method to update the obstacle
// detection status based on laser scan data.
void WalkerContext::updateObstacleDetection(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  float min_distance = std::numeric_limits<float>::infinity();

  // Define the forward angular range
  double forward_angle_min = -M_PI / 6;  // -45 degrees
  double forward_angle_max = M_PI / 6;   // +45 degrees

  // Iterate through the ranges within the forward angular range
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;

    // Check if the angle is within the forward range
    if (angle >= forward_angle_min && angle <= forward_angle_max) {
      float range = msg->ranges[i];
      if (std::isfinite(range) && range < min_distance) {
        min_distance = range;
      }
    }
  }

  // Log the minimum distance within the forward range
  RCLCPP_INFO(node_->get_logger(),
              "Minimum distance from LIDAR (forward): %.2f meters",
              min_distance);

  // Update obstacle detection
  obstacle_detected_ = (min_distance < 0.55);

  // Check if an obstacle is detected and log the status
  if (obstacle_detected_) {
    RCLCPP_WARN(node_->get_logger(), "Obstacle detected at %.2f meters",
                min_distance);
  } else {
    RCLCPP_INFO(node_->get_logger(), "No obstacle detected in forward range.");
  }
}

// Implement the publishCommand() method to publish velocity commands to the
// robot.
void WalkerContext::publishCommand(double linear, double angular) {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);
}

// Add the isObstacleDetected() method to the WalkerContext class to check if an
// obstacle is detected.
bool WalkerContext::isObstacleDetected() const { return obstacle_detected_; }

// Add the shouldRotateClockwise() method to the WalkerContext class to check
// the current rotation direction.
bool WalkerContext::shouldRotateClockwise() const { return rotate_clockwise_; }

// Add the toggleRotationDirection() method to the WalkerContext class to toggle
// the rotation direction.
void WalkerContext::toggleRotationDirection() {
  rotate_clockwise_ = !rotate_clockwise_;
}

// Add the startRotation() method to the WalkerContext class to set the rotation
// start time and flag.
void WalkerContext::startRotation() {
  rotation_start_time_ = clock_->now();
  is_rotating_ = true;
}

// Add the isRotationComplete() method to the WalkerContext class to check if
// the robot has completed the rotation.
bool WalkerContext::isRotationComplete() const {
  if (!is_rotating_) {
    return false;
  }
  // Calculate elapsed time since rotation started
  auto now = clock_->now();
  return (now - rotation_start_time_) >
         rclcpp::Duration(std::chrono::seconds(2));
}

// Add the isRotating() method to the WalkerContext class to check if the robot
// is currently rotating.
bool WalkerContext::isRotating() const { return is_rotating_; }

void WalkerContext::stopRotation() { is_rotating_ = false; }

rclcpp::Logger WalkerContext::getNodeLogger() const {
  return node_->get_logger();
}

// MoveForwardState Implementation
void MoveForwardState::execute(WalkerContext &context) {
  if (context.isObstacleDetected()) {
    // Stop forward motion and transition to RotateState
    context.publishCommand(0.0, 0.0);
    context.setState(std::make_shared<RotateState>());
  } else {
    // Move forward if no obstacle is detected
    context.publishCommand(0.15, 0.0);
  }
}

// RotateState Implementation
void RotateState::execute(WalkerContext &context) {
  if (!context.isRotating()) {
    context.startRotation();  // Start rotation
  }

  double angular = context.shouldRotateClockwise() ? -0.5 : 0.5;

  if (!context.isObstacleDetected()) {
    // Stop rotation and transition back to MoveForwardState
    context.publishCommand(0.0, 0.0);
    context.stopRotation();
    context.toggleRotationDirection();
    RCLCPP_INFO(context.getNodeLogger(),
                "Obstacle cleared. Transitioning to MoveForwardState.");
    context.setState(std::make_shared<MoveForwardState>());
  } else {
    // Continue rotating in the current direction
    context.publishCommand(0.0, angular);
    RCLCPP_INFO(context.getNodeLogger(), "Rotating to avoid obstacle...");
  }
}

// Main function
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create a ROS2 node and initialize the WalkerContext
  auto node = rclcpp::Node::make_shared("walker_node");
  auto walker = std::make_shared<WalkerContext>(node);

  // Set execution loop rate to 10 Hz
  rclcpp::Rate rate(10);

  // Ensure the robot stops on shutdown
  rclcpp::on_shutdown([walker]() { walker->publishCommand(0.0, 0.0); });

  while (rclcpp::ok()) {
    // Execute the current state logic
    walker->execute();

    // Spin the node to process callbacks
    rclcpp::spin_some(node);

    // Sleep to maintain the loop rate
    rate.sleep();
  }

  // Shutdown the ROS2 node
  rclcpp::shutdown();
  return 0;
}
