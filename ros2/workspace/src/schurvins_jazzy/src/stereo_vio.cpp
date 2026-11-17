#include <atomic>
#include <condition_variable>
#include <deque>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <thread>
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <svo_ros/svo_factory.h>
#include <svo/frame_handler_stereo.h>
#include <svo/imu_handler.h>
#include <svo/common/frame.h>
#include <hexbot_vio/tag_initialization.hpp>

class StereoVio : public rclcpp::Node {
  using Image = sensor_msgs::msg::Image;
  using Pair = std::pair<Image::ConstSharedPtr, Image::ConstSharedPtr>;
  using Policy = message_filters::sync_policies::ExactTime<Image, Image>;
 public:
  StereoVio() : Node("schurvins") {
    const auto config_file = declare_parameter<std::string>("config_file", "");
    const auto calibration_file = declare_parameter<std::string>("calibration_file", "");
    if (config_file.empty() || calibration_file.empty())
      throw std::runtime_error("config_file and calibration_file are required");
    auto config = YAML::LoadFile(config_file);
    if (!config["use_imu"] || !config["use_imu"].as<bool>())
      throw std::runtime_error("The stereo VIO adapter requires use_imu=true");
    for (const auto& option : {"runlc", "use_global_map", "use_ceres_backend", "use_backend"})
      if (config[option] && config[option].as<bool>())
        throw std::runtime_error(std::string(option) + " is outside the first stereo VIO build");
    config["calib_file"] = calibration_file;
    const char* out = std::getenv("VIO_OUT");
    const std::string trace_default = std::string(out ? out : "/workspaces/schurvins_ws/_out") + "/trace";
    const auto trace_dir = declare_parameter<std::string>("trace_dir", trace_default);
    if (trace_dir.empty()) throw std::runtime_error("trace_dir must not be empty");
    std::filesystem::create_directories(trace_dir);
    config["trace_dir"] = trace_dir;
    vio_ = svo::factory::makeStereo(config);
    if (vio_->getNCamera()->numCameras() != 2)
      throw std::runtime_error("This adapter requires exactly two calibrated cameras");
    tag_ = std::make_unique<hexbot_vio::TagInitialization>(*this, *vio_->getNCamera());
    imu_ = svo::factory::getImuHandler(config);
    vio_->setImuHandler(imu_);
    vio_->start();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vio/pose", 100);
    status_pub_ = create_publisher<std_msgs::msg::String>("/vio/status", 10);
    const bool reliable = declare_parameter<bool>("input_reliable", false);
    auto qos = rclcpp::QoS(1000);
    if (reliable) qos.reliable(); else qos.best_effort();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu0", qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (stamp <= latest_imu_) {
            RCLCPP_ERROR(get_logger(), "Non-monotonic IMU sample rejected"); return;
          }
          imu_->addImuMeasurement(svo::ImuMeasurement(stamp * 1e-9,
            Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                            msg->linear_acceleration.z)));
          latest_imu_ = stamp;
        }
        ready_.notify_one();
      });
    auto image_policy = rclcpp::QoS(100);
    if (reliable) image_policy.reliable(); else image_policy.best_effort();
    auto image_qos = image_policy.get_rmw_qos_profile();
    left_.subscribe(this, "/cam0/image_raw", image_qos);
    right_.subscribe(this, "/cam1/image_raw", image_qos);
    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(100), left_, right_);
    sync_->registerCallback(&StereoVio::queueImages, this);
    worker_ = std::thread([this] { process(); });
    RCLCPP_INFO(get_logger(), "Stereo VIO ready: /cam{0,1}/image_raw + /imu0 -> /vio/pose");
  }
  ~StereoVio() override {
    stop_ = true; ready_.notify_all();
    if (worker_.joinable()) worker_.join();
  }
 private:
  void queueImages(const Image::ConstSharedPtr& left, const Image::ConstSharedPtr& right) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_.size() >= 100) {
        RCLCPP_ERROR(get_logger(), "Image backlog exceeded 100 pairs; dropping oldest pair");
        pending_.pop_front();
      }
      pending_.emplace_back(left, right);
    }
    ready_.notify_one();
  }
  void process() {
    int64_t last_image = -1;
    try {
      while (!stop_) {
        Pair pair;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          ready_.wait(lock, [this] {
            return stop_ || (!pending_.empty() && latest_imu_ >=
              rclcpp::Time(pending_.front().first->header.stamp).nanoseconds());
          });
          if (stop_) break;
          pair = pending_.front(); pending_.pop_front();
        }
        const auto stamp = rclcpp::Time(pair.first->header.stamp).nanoseconds();
        if (stamp <= last_image) {
          RCLCPP_ERROR(get_logger(), "Non-monotonic stereo pair rejected"); continue;
        }
        last_image = stamp;
        if (!vio_->hasStarted()) {
          svo::Quaternion rotation;
          if (!imu_->getInitialAttitude(stamp * 1e-9, rotation)) continue;
          vio_->setRotationPrior(rotation);
        } else if (vio_->getLastFrames()) {
          svo::Quaternion rotation;
          if (imu_->getRelativeRotationPrior(
                vio_->getLastFrames()->getMinTimestampNanoseconds() * 1e-9,
                stamp * 1e-9, false, rotation)) vio_->setRotationIncrementPrior(rotation);
        }
        auto left = cv_bridge::toCvShare(pair.first, "mono8");
        auto right = cv_bridge::toCvShare(pair.second, "mono8");
        vio_->addImageBundle({left->image, right->image}, stamp);
        std_msgs::msg::String status;
        status.data = vio_->stageStr(); status_pub_->publish(status);
        if (vio_->stage() == svo::Stage::kTracking && vio_->getLastFrames()) {
          auto transform = vio_->getLastFrames()->get_T_W_B();
          auto position = transform.getPosition();
          auto q = transform.getRotation().toImplementation();
          if (!position.allFinite() || !q.coeffs().allFinite())
            throw std::runtime_error("VIO returned a non-finite pose");
          geometry_msgs::msg::PoseStamped msg;
          msg.header = pair.first->header; msg.header.frame_id = "vio_world";
          msg.pose.position.x = position.x(); msg.pose.position.y = position.y();
          msg.pose.position.z = position.z();
          msg.pose.orientation.x = q.x(); msg.pose.orientation.y = q.y();
          msg.pose.orientation.z = q.z(); msg.pose.orientation.w = q.w();
          pose_pub_->publish(msg);
          if (vio_->getLastFrames()->getMinTimestampNanoseconds() == stamp)
            tag_->process(left->image, right->image, rclcpp::Time(msg.header.stamp),
                          Eigen::Isometry3d(transform.getTransformationMatrix()));
        } else if (vio_->stage() == svo::Stage::kPaused || vio_->stage() == svo::Stage::kInitializing) {
          tag_->reset();
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "VIO stopped: %s", e.what());
      rclcpp::shutdown();
    }
  }
  std::shared_ptr<svo::FrameHandlerStereo> vio_;
  std::unique_ptr<hexbot_vio::TagInitialization> tag_;
  std::shared_ptr<svo::ImuHandler> imu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  message_filters::Subscriber<Image> left_, right_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Pair> pending_;
  int64_t latest_imu_ = -1;
  std::atomic<bool> stop_{false};
  std::thread worker_;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<StereoVio>()); }
  catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
  rclcpp::shutdown();
  return 0;
}
