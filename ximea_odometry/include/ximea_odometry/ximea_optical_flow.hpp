#ifndef XIMEA_ODOMETRY__XIMEA_OPTICAL_FLOW_NODE_HPP_
#define XIMEA_ODOMETRY__XIMEA_OPTICAL_FLOW_NODE_HPP_

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int16.hpp"

// Exposue: 1865, Gain 20

class XimeaOpticalFlowNode : public rclcpp::Node {
 public:
  explicit XimeaOpticalFlowNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  //   ~XimeaOpticalFlowNode();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info;

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  cv::Mat current_frame;
};

#endif  // XIMEA_ODOMETRY__XIMEA_OPTICAL_FLOW_NODE_HPP_
