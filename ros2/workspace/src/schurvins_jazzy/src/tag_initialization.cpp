#include <hexbot_vio/tag_initialization.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <vikit/cameras/camera_geometry_base.h>

namespace hexbot_vio {
namespace {
using Pose = Eigen::Isometry3d;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Pixels = std::vector<cv::Point2f>;

constexpr double radians = 0.017453292519943295;
constexpr double chi95 = 12.591587;  // Six-dimensional Gaussian confidence ellipsoid.
constexpr double maximum_rmse_px = 2.0;
constexpr int64_t observation_period_ns = 200'000'000;
constexpr int64_t window_ns = 4'000'000'000;
constexpr size_t max_views = 8;

struct View {
  int64_t stamp;
  Pose odom_body;
  std::array<Pixels, 2> pixels;
};

struct Fit {
  Pose tag_odom;
  double cost;
};

bool positive(double x) { return std::isfinite(x) && x > 0.0; }

Pose perturb(const Pose& reference, const cv::Mat& x) {
  Pose pose = reference;
  pose.translation() += Eigen::Vector3d(x.at<double>(0), x.at<double>(1), x.at<double>(2));
  const Eigen::Vector3d w(x.at<double>(3), x.at<double>(4), x.at<double>(5));
  const double angle = w.norm();
  if (angle > 0.) pose.linear() = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix() * reference.linear();
  return pose;
}

double angular_distance(const Pose& a, const Pose& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

void write_pose(const Pose& pose, geometry_msgs::msg::Pose& msg) {
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
}

class Reprojection final : public cv::LMSolver::Callback {
 public:
  Reprojection(const vk::cameras::NCamera& rig, const std::array<Pose, 2>& camera_body,
               const std::vector<cv::Point3d>& corners, const std::deque<View>& views,
               const Pose& reference)
      : rig_(rig), camera_body_(camera_body), corners_(corners), views_(views), reference_(reference) {}

  bool compute(cv::InputArray input, cv::OutputArray error, cv::OutputArray jacobian) const override {
    const cv::Mat x = input.getMat();
    cv::Mat residual;
    if (!evaluate(perturb(reference_, x), residual)) return false;
    residual.copyTo(error);
    if (!jacobian.needed()) return true;
    jacobian.create(residual.rows, 6, CV_64F);
    auto derivative = jacobian.getMat();
    for (int axis = 0; axis < 6; ++axis) {
      constexpr double h = 1e-6;
      cv::Mat shifted = x.clone(), upper, lower;
      shifted.at<double>(axis) += h;
      if (!evaluate(perturb(reference_, shifted), upper)) return false;
      shifted.at<double>(axis) -= 2 * h;
      if (!evaluate(perturb(reference_, shifted), lower)) return false;
      cv::Mat column = (upper - lower) / (2 * h);
      column.copyTo(derivative.col(axis));
    }
    return true;
  }

 private:
  bool evaluate(const Pose& tag_odom, cv::Mat& residual) const {
    residual.create(static_cast<int>(16 * views_.size()), 1, CV_64F);
    int row = 0;
    for (const auto& view : views_) {
      const Pose body_tag = (tag_odom * view.odom_body).inverse();
      for (size_t camera = 0; camera < 2; ++camera) {
        const Pose camera_tag = camera_body_[camera] * body_tag;
        for (size_t i = 0; i < corners_.size(); ++i) {
          const auto& c = corners_[i];
          const Eigen::Vector3d point = camera_tag * Eigen::Vector3d(c.x, c.y, c.z);
          if (!point.allFinite() || point.z() <= 1e-4) return false;
          Eigen::Vector2d uv;
          rig_.getCamera(camera).project3(point, &uv);
          if (!uv.allFinite()) return false;
          residual.at<double>(row++) = uv.x() - view.pixels[camera][i].x;
          residual.at<double>(row++) = uv.y() - view.pixels[camera][i].y;
        }
      }
    }
    return true;
  }

  const vk::cameras::NCamera& rig_;
  const std::array<Pose, 2>& camera_body_;
  const std::vector<cv::Point3d>& corners_;
  const std::deque<View>& views_;
  Pose reference_;
};
}  // namespace

class TagInitialization::Impl {
 public:
  Impl(rclcpp::Node& node, const vk::cameras::NCamera& rig) : node_(node), rig_(rig) {
    id_ = node.declare_parameter<int>("tag_id", -1);
    if (id_ == -1) return;
    const double size = node.declare_parameter<double>("tag_size_m", 0.);
    frame_ = node.declare_parameter<std::string>("tag_frame", "tag_world");
    position_tolerance_ = node.declare_parameter<double>("tag_position_tolerance_m", 0.06);
    rotation_tolerance_ = radians * node.declare_parameter<double>("tag_angle_tolerance_deg", 5.);
    pixel_sigma_ = node.declare_parameter<double>("tag_pixel_sigma_px", 1.);
    position_floor_ = node.declare_parameter<double>("tag_position_floor_m", .01);
    rotation_floor_ = radians * node.declare_parameter<double>("tag_angle_floor_deg", .5);
    if (id_ < 0 || id_ >= 587 || !positive(size) || !positive(position_tolerance_) ||
        !positive(rotation_tolerance_) || !positive(pixel_sigma_) || !positive(position_floor_) ||
        !positive(rotation_floor_) || frame_.empty() || frame_.front() == '/' || frame_ == "vio_world")
      throw std::invalid_argument("Invalid single-Tag initialization parameters (family tag36h11)");
    const double half = size / 2.;
    corners_ = {{-half, half, 0.}, {half, half, 0.}, {half, -half, 0.}, {-half, -half, 0.}};
    for (size_t c = 0; c < 2; ++c) camera_body_[c] = Pose(rig_.get_T_C_B(c).getTransformationMatrix());
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    detector_ = cv::aruco::DetectorParameters::create();
    detector_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/vio/tag_pose", 10);
    alignment_pub_ = node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/vio/tag_alignment", 10);
    status_pub_ = node.create_publisher<std_msgs::msg::String>("/vio/tag_status", rclcpp::QoS(1).transient_local());
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    state("waiting_for_tag");
  }

  void reset() {
    if (id_ < 0) return;
    views_.clear();
    committed_.reset();
    last_attempt_ = -1;
    state("waiting_for_tag");
  }

  void process(const cv::Mat& left, const cv::Mat& right, const rclcpp::Time& stamp, const Pose& odom_body) {
    if (id_ < 0) return;
    if (!committed_ && (last_attempt_ < 0 || stamp.nanoseconds() - last_attempt_ >= observation_period_ns)) {
      last_attempt_ = stamp.nanoseconds();
      try {
        observe({left, right}, stamp, odom_body);
      } catch (const cv::Exception& error) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                             "Tag observation declined: %s", error.what());
        state("insufficient_information");
      }
    }
    if (!committed_) return;
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_;
    write_pose(*committed_ * odom_body, msg.pose);
    pose_pub_->publish(msg);
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = "vio_world";
    tf.transform.translation.x = committed_->translation().x();
    tf.transform.translation.y = committed_->translation().y();
    tf.transform.translation.z = committed_->translation().z();
    const Eigen::Quaterniond q(committed_->linear());
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_->sendTransform(tf);
  }

 private:
  void state(const std::string& value) {
    if (state_ == value) return;
    state_ = value;
    std_msgs::msg::String msg;
    msg.data = value;
    status_pub_->publish(msg);
  }

  bool detect(const cv::Mat& image, Pixels& result) const {
    std::vector<int> ids;
    std::vector<Pixels> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, detector_);
    const auto match = std::find(ids.begin(), ids.end(), id_);
    if (match == ids.end() || std::find(std::next(match), ids.end(), id_) != ids.end()) return false;
    const auto& pixels = corners[std::distance(ids.begin(), match)];
    for (size_t i = 0; i < 4; ++i) {
      if (cv::norm(pixels[i] - pixels[(i + 1) % 4]) < 20.) return false;
    }
    result = pixels;
    return true;
  }

  std::vector<Pose> seeds(const View& view) const {
    std::vector<cv::Point2d> normalized;
    for (const auto& px : view.pixels[0]) {
      Eigen::Vector3d ray;
      if (!rig_.getCamera(0).backProject3(Eigen::Vector2d(px.x, px.y), &ray) || !ray.allFinite() ||
          ray.z() <= 0.) return {};
      normalized.emplace_back(ray.x() / ray.z(), ray.y() / ray.z());
    }
    std::vector<cv::Mat> rotation, translation;
    cv::solvePnPGeneric(corners_, normalized, cv::Matx33d::eye(), cv::noArray(),
                        rotation, translation, false, cv::SOLVEPNP_IPPE_SQUARE);
    std::vector<Pose> starts;
    for (size_t i = 0; i < rotation.size(); ++i) {
      cv::Mat tangent = cv::Mat::zeros(6, 1, CV_64F);
      translation[i].copyTo(tangent.rowRange(0, 3));
      rotation[i].copyTo(tangent.rowRange(3, 6));
      if (cv::checkRange(tangent)) {
        const Pose camera_tag = perturb(Pose::Identity(), tangent);
        starts.emplace_back((view.odom_body * camera_body_[0].inverse() * camera_tag).inverse());
      }
    }
    return starts;
  }

  void observe(const std::array<cv::Mat, 2>& images, const rclcpp::Time& stamp, const Pose& odom_body) {
    while (!views_.empty() && stamp.nanoseconds() - views_.front().stamp > window_ns) views_.pop_front();
    View view{stamp.nanoseconds(), odom_body, {}};
    if (!detect(images[0], view.pixels[0]) || !detect(images[1], view.pixels[1])) {
      state("waiting_for_tag");
      return;
    }
    if (state_ == "waiting_for_tag") state("estimating");
    // Near-identical geometries share corner bias and must not tighten the confidence gate.
    if (std::any_of(views_.begin(), views_.end(), [&view](const View& old) {
          return (old.odom_body.translation() - view.odom_body.translation()).norm() < .035 &&
                 angular_distance(old.odom_body, view.odom_body) < 4 * radians;
        })) {
      state("insufficient_information");
      return;
    }
    auto starts = seeds(view);
    if (starts.empty()) {
      state("pose_fit_rejected");
      return;
    }
    auto window = views_;
    window.push_back(std::move(view));
    if (window.size() > max_views) window.pop_front();

    std::vector<Fit> candidates;
    for (const auto& start : starts) {
      auto residual = cv::makePtr<Reprojection>(rig_, camera_body_, corners_, window, start);
      cv::Mat x = cv::Mat::zeros(6, 1, CV_64F), before, after;
      if (!residual->compute(x, before, cv::noArray())) continue;
      cv::LMSolver::create(residual, 18)->run(x);
      if (!cv::checkRange(x) || !residual->compute(x, after, cv::noArray())) continue;
      bool valid = true;
      for (int row = 0; row < after.rows; row += 16) {
        if (cv::norm(after.rowRange(row, row + 16), cv::NORM_L2SQR) > 8 * maximum_rmse_px * maximum_rmse_px)
          valid = false;
      }
      if (valid) candidates.push_back({perturb(start, x), cv::norm(after, cv::NORM_L2SQR)});
    }
    if (candidates.empty()) {
      state("pose_fit_rejected");
      return;
    }
    views_ = std::move(window);
    std::sort(candidates.begin(), candidates.end(), [](const Fit& a, const Fit& b) { return a.cost < b.cost; });
    const auto& best = candidates.front();
    for (const auto& alternative : candidates) {
      if (alternative.cost - best.cost < 9 * pixel_sigma_ * pixel_sigma_ &&
          ((alternative.tag_odom.translation() - best.tag_odom.translation()).norm() > position_tolerance_ / 2 ||
           angular_distance(alternative.tag_odom, best.tag_odom) > rotation_tolerance_ / 2)) {
        state("ambiguous_pose");
        return;
      }
    }

    Reprojection objective(rig_, camera_body_, corners_, views_, best.tag_odom);
    cv::Mat error, jacobian;
    if (!objective.compute(cv::Mat::zeros(6, 1, CV_64F), error, jacobian)) return;
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>> j(
        jacobian.ptr<double>(), jacobian.rows, 6);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(j, Eigen::ComputeThinV);
    if (svd.singularValues()(5) <= svd.singularValues()(0) * 1e-8) {
      state("insufficient_information");
      return;
    }
    Matrix6 covariance = svd.matrixV() *
        (pixel_sigma_ * pixel_sigma_ * svd.singularValues().array().square().inverse()).matrix().asDiagonal() *
        svd.matrixV().transpose();
    // Check positional uncertainty at the body, not only at the VIO origin.
    const Eigen::Vector3d lever = best.tag_odom.linear() * odom_body.translation();
    Eigen::Matrix3d cross;
    cross << 0., -lever.z(), lever.y(), lever.z(), 0., -lever.x(), -lever.y(), lever.x(), 0.;
    Matrix6 body_jacobian = Matrix6::Identity();
    body_jacobian.topRightCorner<3, 3>() = -cross;
    covariance.diagonal().head<3>().array() += position_floor_ * position_floor_;
    covariance.diagonal().tail<3>().array() += rotation_floor_ * rotation_floor_;
    Vector6 scale;
    scale.head<3>().setConstant(1. / position_tolerance_);
    scale.tail<3>().setConstant(1. / rotation_tolerance_);
    const Matrix6 normalized = scale.asDiagonal() * body_jacobian * covariance *
        body_jacobian.transpose() * scale.asDiagonal();
    if (!normalized.allFinite()) {
      state("insufficient_information");
      return;
    }
    Eigen::SelfAdjointEigenSolver<Matrix6> eigenvalues(normalized, Eigen::EigenvaluesOnly);
    if (eigenvalues.info() != Eigen::Success) {
      state("insufficient_information");
      return;
    }
    const double bound = std::sqrt(chi95 * eigenvalues.eigenvalues().maxCoeff());
    RCLCPP_DEBUG(node_.get_logger(), "Tag %d: %zu distinct views, normalized 95%% bound %.3f",
                 id_, views_.size(), bound);
    if (!std::isfinite(bound) || bound > 1.) {
      state("insufficient_information");
      return;
    }

    committed_ = best.tag_odom;
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_;
    write_pose(*committed_, msg.pose.pose);
    for (int i = 0; i < 6; ++i)
      for (int k = 0; k < 6; ++k) msg.pose.covariance[6 * i + k] = covariance(i, k);
    alignment_pub_->publish(msg);
    RCLCPP_INFO(node_.get_logger(), "Tag %d alignment committed at %.3f s using %zu distinct views (normalized 95%% bound %.3f)",
                id_, stamp.seconds(), views_.size(), bound);
    views_.clear();
    state("ready");
  }

  rclcpp::Node& node_;
  const vk::cameras::NCamera& rig_;
  int id_;
  int64_t last_attempt_ = -1;
  double position_tolerance_, rotation_tolerance_, pixel_sigma_, position_floor_, rotation_floor_;
  std::string frame_, state_;
  std::vector<cv::Point3d> corners_;
  std::deque<View> views_;
  std::array<Pose, 2> camera_body_;
  std::optional<Pose> committed_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr alignment_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
};

TagInitialization::TagInitialization(rclcpp::Node& node, const vk::cameras::NCamera& cameras)
    : impl_(std::make_unique<Impl>(node, cameras)) {}
TagInitialization::~TagInitialization() = default;

void TagInitialization::process(const cv::Mat& left, const cv::Mat& right, const rclcpp::Time& stamp,
                                const Eigen::Isometry3d& odom_body) {
  impl_->process(left, right, stamp, odom_body);
}

void TagInitialization::reset() { impl_->reset(); }
}  // namespace hexbot_vio
