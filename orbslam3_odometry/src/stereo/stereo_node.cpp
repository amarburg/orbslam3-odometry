#include <signal.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include "System.h"
#include "rclcpp/rclcpp.hpp"
#include "stereo.hpp"

#define DEBUG false

void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cerr << "Received SIGINT. Killing StereoSlamNode process."
              << std::endl;
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, handleSignal);

  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StereoSlamNode>();
  RCLCPP_INFO(node->get_logger(), "Created Node");
  RCLCPP_INFO(node->get_logger(), "============================ ");
  RCLCPP_INFO(node->get_logger(), "Spinning");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
