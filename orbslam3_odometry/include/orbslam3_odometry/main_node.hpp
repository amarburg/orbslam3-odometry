#include <signal.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include "System.h"
#include "rclcpp/rclcpp.hpp"

#define DEBUG false

class MainNode : public rclcpp::Node {
 public:
  MainNode(const std::string &node_name) : Node(node_name) {
    /* ***** DECLARING PARAMETERS ***** */

    this->declare_parameter("pangolin_visualization", true);
    this->declare_parameter("path_vocabulary", "");
    this->declare_parameter("path_yaml_settings", "");

    /* ******************************** */

    /* ***** READING PARAMETERS ***** */

    this->get_parameter("pangolin_visualization", pangolin_visualization);
    this->get_parameter("path_vocabulary", path_vocabulary);
    this->get_parameter("path_yaml_settings", path_settings);

    if (DEBUG) printDebug();

    // if (just_check_stereo_calibration || just_take_picture) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "CHECKING STEREO RECTIFICATION OR TAKING PICTURE. ORB-SLAM
    //               " "WILL NOT START");

    //   auto node =
    //   std::make_shared<JustCheckStereoCalibration>(path_settings);
    //   rclcpp::spin(node);
    //   return;
    // } else
    //   RCLCPP_INFO(this->get_logger(), "NON CHECKING STEREO RECTIFICATION");

    // if (system_mode == "mono") {
    //   ORB_SLAM3::System pSLAM(path_vocabulary, path_settings,
    //                           ORB_SLAM3::System::MONOCULAR,
    //                           pangolin_visualization);
    //   auto node = std::make_shared<MonocularSlamNode>(&pSLAM);
    //   rclcpp::spin(node);
    //   return;
    // } else if (system_mode == "stereo") {
    //   ORB_SLAM3::System pSLAM(path_vocabulary, path_settings,
    //                           ORB_SLAM3::System::STEREO,
    //                           pangolin_visualization);
    //   auto node = std::make_shared<StereoSlamNode>(&pSLAM, path_settings);
    //   rclcpp::spin(node);
    //   return;
    // }
  }

 protected:
  bool pangolin_visualization;
  std::string path_vocabulary, path_settings, system_mode;

  void printDebug() {
    RCLCPP_INFO(this->get_logger(), "ORB Voc %s", path_vocabulary.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings %s", path_settings.c_str());
    RCLCPP_INFO(this->get_logger(), "Pangolin: %s",
                pangolin_visualization ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "System mode: %s", system_mode.c_str());
  }
};
