/**
 * @brief Header file for the walker states and context classes.
 * @file walker_states.hpp
 * @date November 19 2024
 * @version 1.5
 * @author Swaraj Mundruppady Rao (swarajmr@umd.edu)
 * @copyright Copyright (c) 2024
 */
#ifndef WALKER_STATES_HPP
#define WALKER_STATES_HPP

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief Abstract base class for the states in the walker algorithm.
 */
class WalkerState {
 public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~WalkerState() = default;

  /**
   * @brief Pure virtual method for executing the state logic.
   * @param context Reference to the WalkerContext for state interaction.
   */
  virtual void execute(class WalkerContext &context) = 0;
};

/**
 * @brief Context class for the walker algorithm, managing state transitions and
 * robot actions.
 */
class WalkerContext {
 private:
  rclcpp::Node::SharedPtr node_;  ///< Pointer to the ROS2 node.
  std::shared_ptr<WalkerState>
      current_state_;  ///< Current state of the walker.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_pub_;  ///< Publisher for velocity commands.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_sub_;                   ///< Subscription to laser scan data.
  rclcpp::Clock::SharedPtr clock_;  ///< Clock for managing timed operations.
  bool obstacle_detected_;  ///< Flag indicating if an obstacle is detected.
  bool rotate_clockwise_;   ///< Flag indicating rotation direction.
  rclcpp::Time rotation_start_time_;  ///< Time when the rotation started.
  bool is_rotating_;  ///< Flag indicating if the robot is currently rotating.

 public:
  /**
   * @brief Constructor to initialize the WalkerContext.
   * @param node Shared pointer to the ROS2 node.
   */
  WalkerContext(rclcpp::Node::SharedPtr node);

  /**
   * @brief Set the current state of the walker.
   * @param state Shared pointer to the new state.
   */
  void setState(std::shared_ptr<WalkerState> state);

  /**
   * @brief Execute the logic of the current state.
   */
  void execute();

  /**
   * @brief Update the obstacle detection status based on laser scan data.
   * @param msg Shared pointer to the laser scan message.
   */
  void updateObstacleDetection(
      const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /**
   * @brief Publish velocity commands to the robot.
   * @param linear Linear velocity.
   * @param angular Angular velocity.
   */
  void publishCommand(double linear, double angular);

  /**
   * @brief Check if an obstacle is detected.
   * @return True if an obstacle is detected, false otherwise.
   */
  bool isObstacleDetected() const;

  /**
   * @brief Check the current rotation direction.
   * @return True if rotating clockwise, false otherwise.
   */
  bool shouldRotateClockwise() const;

  /**
   * @brief Toggle the rotation direction.
   */
  void toggleRotationDirection();

  /**
   * @brief Start the rotation timer.
   */
  void startRotation();

  /**
   * @brief Check if the rotation is complete.
   * @return True if the rotation duration has elapsed, false otherwise.
   */
  bool isRotationComplete() const;

  /**
   * @brief Stop the rotation.
   */
  void stopRotation();

  /**
   * @brief Get the logger for the ROS2 node.
   * @return Logger object for logging.
   */
  rclcpp::Logger getNodeLogger() const;

  /**
   * @brief Check if the robot is currently rotating.
   * @return True if the robot is rotating, false otherwise.
   */
  bool isRotating() const;
};

/**
 * @brief Concrete state class for moving forward.
 */
class MoveForwardState : public WalkerState {
 public:
  /**
   * @brief Execute the logic for moving forward.
   * @param context Reference to the WalkerContext.
   */
  void execute(WalkerContext &context) override;
};

/**
 * @brief Concrete state class for rotating.
 */
class RotateState : public WalkerState {
 public:
  /**
   * @brief Execute the logic for rotating.
   * @param context Reference to the WalkerContext.
   */
  void execute(WalkerContext &context) override;
};

#endif  // WALKER_STATES_HPP
