#include "ximea_odometry/ximea_optical_flow.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include <image_transport/image_transport.hpp>

XimeaOpticalFlowNode::XimeaOpticalFlowNode(const rclcpp::NodeOptions& options)
    : Node("ximea_feature_detection_node", options) {
  // Initialize subscribers
  camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", 10,
      std::bind(&XimeaOpticalFlowNode::camera_info_callback, this, std::placeholders::_1));

  image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10, std::bind(&XimeaOpticalFlowNode::image_callback, this, std::placeholders::_1));

  // For testing with one frame
  current_frame = cv::imread("frame.png", cv::IMREAD_COLOR);
  cv::imshow("new window", current_frame);
}

void XimeaOpticalFlowNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    current_frame = cv_ptr->image;

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  return;
}

void XimeaOpticalFlowNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  auto camera_info = msg;
  return;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XimeaOpticalFlowNode>());
  rclcpp::shutdown();
  return 0;
}