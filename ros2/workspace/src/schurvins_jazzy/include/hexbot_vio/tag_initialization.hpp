#pragma once

#include <memory>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vikit/cameras/ncamera.h>

namespace hexbot_vio {

class TagInitialization {
 public:
  TagInitialization(rclcpp::Node& node, const vk::cameras::NCamera& cameras);
  ~TagInitialization();

  void process(const cv::Mat& left, const cv::Mat& right, const rclcpp::Time& stamp,
               const Eigen::Isometry3d& odom_body);
  // A different odometry epoch requires a fresh map alignment.
  void reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hexbot_vio
