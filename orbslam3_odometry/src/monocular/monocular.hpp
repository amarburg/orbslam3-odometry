#pragma once

#include "orbslam3_odometry/main_node.hpp"
#include "orbslam3_odometry/utility.hpp"

// ORBSLAM header files
#include "Frame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"
#include "cv_bridge/cv_bridge.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using sensor_msgs::msg::CameraInfo;

class MonocularSlamNode : public MainNode {
 public:
  MonocularSlamNode();

  ~MonocularSlamNode();

 private:
  void syncedCallback(const ImageMsg::ConstSharedPtr &image_msg,
                      const CameraInfo::ConstSharedPtr &camera_info);

  std::shared_ptr<ORB_SLAM3::System> m_SLAM;

  cv_bridge::CvImagePtr m_cvImPtr;

  // Images stuff
  message_filters::Subscriber<ImageMsg> subscription_img;
  message_filters::Subscriber<CameraInfo> subscription_info;

  typedef message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<ImageMsg, CameraInfo> >
      MonocularImageSync;
  std::shared_ptr<MonocularImageSync> sync_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr quaternion_pub;

  void loadParameters();

  /*List of all parameters */
  std::string camera_left_topic, camera_right_topic;
  std::string imu, header_id_frame, child_id_frame;
  std::string camera_left_info_topic, camera_right_info_topic;

  std::string topic_pub_quat;
  bool isCameraLeft;
  int scale_position_mono;
  int degree_move_pose_mono;

  // cropping parameters
  int cropping_x, cropping_y, cropping_width, cropping_height;
  cv::Rect cropping_rect;

  // Point cloud and Key points varables/methods
  std::vector<float> depths;
  sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(
      std::vector<ORB_SLAM3::MapPoint *> map_points, rclcpp::Time msg_time,
      Eigen::Vector3f actualPosition);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisherPointCloud;
  cv::Scalar interpolateColor(float value, float minDepth, float maxDepth);
};
