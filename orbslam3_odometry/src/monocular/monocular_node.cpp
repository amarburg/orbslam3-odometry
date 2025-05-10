#include <signal.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include "System.h"
#include "monocular.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEBUG false

void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cout << "Received SIGINT. Killing MonocularSlamNode process."
              << std::endl;
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, handleSignal);

  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MonocularSlamNode>();

  std::cout << "============================ " << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
