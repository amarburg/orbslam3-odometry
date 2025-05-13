#include <sys/stat.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "orbslam3_odometry/main_node.hpp"
#include "orbslam3_odometry/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_rectification.hpp"

using namespace std::chrono_literals;

using sensor_msgs::msg::CameraInfo;

class JustCheckStereoCalibration : public MainNode {
 public:
  JustCheckStereoCalibration() : MainNode("orbslam3_odometry") {
    this->loadParameters();

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();

    if (isTakingPicture) {
      if (!directory_exists(path) || !directory_exists(pathLeft) ||
          !directory_exists(pathRight)) {
        RCLCPP_ERROR_STREAM(
            this->get_logger(),
            "Directory " << path << " oppure " << pathLeft << " oppure "
                         << pathRight << " Non essitono. Crearle: mkdir -p "
                         << pathLeft << " ; mkdir -p " << pathRight);
        exit(1);
      }

      RCLCPP_INFO(
          this->get_logger(),
          "JUST TAKING PICTURE. IMAGES WILL BE READ FROM TOPICS %s and %s",
          this->camera_left_topic.c_str(), this->camera_right_topic.c_str());
    } else {
      // Read the parametes and compute the common_roi, so that cropped and
      // rectified images will have the same size
      readParameters(path_settings, map1_L, map2_L, roi_L, map1_R, map2_R,
                     roi_R);
      common_roi = roi_L & roi_R;
      RCLCPP_INFO(this->get_logger(),
                  "CHECKING STEREO RECTIFICATION STARTED. IMAGES WILL BE READ "
                  "FROM TOPICS %s and %s",
                  this->camera_left_topic.c_str(),
                  this->camera_right_topic.c_str());
    }
    Utility::printCommonInfo(qos);

    subscription_left.subscribe(this, this->camera_left_topic,
                                qos.get_rmw_qos_profile());
    subscription_right.subscribe(this, this->camera_right_topic,
                                 qos.get_rmw_qos_profile());

    subscription_left_info.subscribe(this, this->camera_left_info_topic,
                                     qos.get_rmw_qos_profile(),
                                     rclcpp::SubscriptionOptions());
    subscription_right_info.subscribe(this, this->camera_right_info_topic,
                                      qos.get_rmw_qos_profile(),
                                      rclcpp::SubscriptionOptions());

    uint32_t queue_size = 10;
    sync_ = std::make_shared<StereoImageSync>(
        message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                        CameraInfo, CameraInfo>(
            queue_size),
        subscription_left, subscription_right, subscription_left_info,
        subscription_right_info);

    sync_->setAgePenalty(1.00);
    sync_->registerCallback(std::bind(
        &JustCheckStereoCalibration::syncCallback, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    cv::namedWindow("Synchronized Images", cv::WINDOW_NORMAL);

    // cv::namedWindow("Left NON Rectified", cv::WINDOW_NORMAL);
    // cv::namedWindow("Right NON Rectified", cv::WINDOW_NORMAL);
    saved_image_count = 0;

    //        sync_timer_ = this->create_wall_timer(100ms,
    //        std::bind(&JustCheckStereoCalibration::SyncImages, this));
  }

 private:
  void loadParameters() {
    declare_parameter("topic_camera_left", "/camera/left_image");
    declare_parameter("topic_camera_right", "/camera/right_image");

    declare_parameter("topic_camera_info_left", "/camera/left_camera_info");
    declare_parameter("topic_camera_info_right", "/camera/right_camera_info");

    declare_parameter("just_take_picture", false);

    get_parameter("topic_camera_left", this->camera_left_topic);
    get_parameter("topic_camera_right", this->camera_right_topic);
    get_parameter("topic_camera_info_left", this->camera_left_info_topic);
    get_parameter("topic_camera_info_right", this->camera_right_info_topic);
    get_parameter("just_take_picture", this->isTakingPicture);
  }

  bool directory_exists(const std::string &dir) {
    struct stat info;
    return stat(dir.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
  }

  void syncCallback(const ImageMsg::ConstSharedPtr &left_imagemsg,
                    const ImageMsg::ConstSharedPtr &right_imagemsg,
                    const CameraInfo::ConstSharedPtr &left_info,
                    const CameraInfo::ConstSharedPtr &right_info) {
    cv::Mat left_image, right_image;

    try {
      left_image = cv_bridge::toCvShare(left_imagemsg, "bgr8")->image;
      right_image = cv_bridge::toCvShare(right_imagemsg, "bgr8")->image;

      // cv::imshow("Left NON Rectified", left_image);
      // cv::imshow("Right NON Rectified", right_image);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (!isTakingPicture) {
      // isTakingPicture is false, so i will show rectification results and
      // disparity map

      // Perform stereo rectification. I save also the rectified but not cropped
      // images
      //               rectify_image(left_image, map1_L, map2_L, common_roi,
      //               img_non_cropped_L); rectify_image(right_image, map1_R,
      //               map2_R, common_roi, img_non_cropped_R);

      // Draw horizontal lines on rectified images
      const int DISTANZA_RIGHE_PIXEL = 50;

      for (int y = 0; y < left_image.rows; y += DISTANZA_RIGHE_PIXEL) {
        cv::line(left_image, cv::Point(0, y), cv::Point(left_image.cols - 1, y),
                 cv::Scalar(0, 0, 255), 1);
      }

      for (int y = 0; y < right_image.rows; y += DISTANZA_RIGHE_PIXEL) {
        cv::line(right_image, cv::Point(0, y),
                 cv::Point(right_image.cols - 1, y), cv::Scalar(0, 0, 255), 1);
      }

      // Perform stereo matching
      show_disparity(left_image, right_image);
      cv::waitKey(1);

    } else {
      // Save images

      // Show images (resized)
      cv::Mat concatenated;
      cv::Mat resizedImage1, resizedImage2;
      cv::resize(left_image, resizedImage1,
                 cv::Size(left_image.cols / 2, left_image.rows / 2));
      cv::resize(right_image, resizedImage2,
                 cv::Size(left_image.cols / 2, left_image.rows / 2));
      cv::hconcat(resizedImage1, resizedImage2, concatenated);
      cv::imshow("Left + Right ", concatenated);

      // Save images if spacebar is pressed
      int key = cv::waitKey(1);
      if (key == 32) {  // Spacebar was pressed
        // watch -n1 ls -Rl /home/formula-student/immagini

        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Salvo immagini. CONTROLLA CHE IL PATH "
                               << path
                               << " + SOTTOCARTELLE left E right ESISTANO");
        cv::imwrite(pathLeft + std::to_string(saved_image_count) + ".jpg",
                    left_image);
        cv::imwrite(pathRight + std::to_string(saved_image_count) + ".jpg",
                    right_image);
        saved_image_count++;

        // Clear matrixes so they won't be re-processed
        left_image.release();
        right_image.release();
      }
    }
  }

  // Rectification output params
  cv::Mat map1_L, map2_L, map1_R, map2_R;
  cv::Rect roi_L, roi_R, common_roi;

  std::string camera_left_topic, camera_right_topic, camera_left_info_topic,
      camera_right_info_topic;
  ;
  bool isTakingPicture;

  message_filters::Subscriber<ImageMsg> subscription_left;
  message_filters::Subscriber<ImageMsg> subscription_right;
  message_filters::Subscriber<CameraInfo> subscription_left_info;
  message_filters::Subscriber<CameraInfo> subscription_right_info;

  typedef message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg,
                                                      CameraInfo, CameraInfo> >
      StereoImageSync;
  std::shared_ptr<StereoImageSync> sync_;

  // cv::Mat left_image, right_image;

  rclcpp::TimerBase::SharedPtr sync_timer_;

  int saved_image_count;

  const std::string path = "/home/formula-student/immagini";
  const std::string pathLeft = path + "/left/";
  const std::string pathRight = path + "/right/";
};

//----------------------------

void handleSignal(int signal) {
  if (signal == SIGINT) {
    std::cout << "Received SIGINT. Killing MainNode process." << std::endl;
    rclcpp::shutdown();
  }
}

int main(int argc, char **argv) {
  signal(SIGINT, handleSignal);

  /* node initialization */
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JustCheckStereoCalibration>();

  std::cout << "============================ " << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
