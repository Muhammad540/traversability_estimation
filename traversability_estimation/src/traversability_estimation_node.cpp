/*
 * traversability_estimation_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 *   
 *  PORTED TO ROS2 HUMBLE BY : Muhammad Ahmed
 */

#include "rclcpp/rclcpp.hpp"
#include "traversability_estimation/TraversabilityEstimation.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto nodeHandle = std::make_shared<rclcpp::Node>("traversability_estimation");
  traversability_estimation::TraversabilityEstimation traversabilityEstimation(nodeHandle);

  // Spin
  rclcpp::spin(nodeHandle);
  rclcpp::shutdown();

  return 0;
}
