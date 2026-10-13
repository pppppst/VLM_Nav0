// Camera mount TF ownership only; no sensors, navigation or motion commands.
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace std::chrono_literals;

class CameraTF : public rclcpp::Node {
 public:
  CameraTF() : Node("go2_camera_tf") {
    transform_.header.frame_id = declare_parameter<std::string>("parent_frame", "");
    transform_.child_frame_id = declare_parameter<std::string>("child_frame", "");
    auto xyz = declare_parameter<std::vector<double>>("xyz", std::vector<double>{});
    auto rpy = declare_parameter<std::vector<double>>("rpy", std::vector<double>{});
    if (transform_.header.frame_id != "base_link" || transform_.child_frame_id != "camera_link" ||
        xyz.size() != 3 || rpy.size() != 3) throw std::runtime_error("camera mount must be base_link -> camera_link with xyz/rpy");
    for (const auto & values : {xyz, rpy}) for (double v : values)
      if (!std::isfinite(v)) throw std::runtime_error("camera xyz/rpy must be finite");
    auto & t = transform_.transform.translation;
    t.x = xyz[0]; t.y = xyz[1]; t.z = xyz[2];
    tf2::Quaternion q; q.setRPY(rpy[0], rpy[1], rpy[2]);
    auto & r = transform_.transform.rotation;
    r.x = q.x(); r.y = q.y(); r.z = q.z(); r.w = q.w();
    auto domain = get_node_base_interface()->get_context()->get_domain_id();
    auto path = "/tmp/go2-camera-tf-" + std::to_string(getuid()) + "-" +
      std::to_string(domain) + "-base-camera.lock";
    lease_ = open(path.c_str(), O_CREAT | O_RDWR | O_NOFOLLOW, 0600);
    if (lease_ < 0) throw std::runtime_error("cannot open camera TF ownership lock");
    auto callback = [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
      auto id = gid(info.get_rmw_message_info().publisher_gid.data);
      if (owned_.count(id)) return;
      for (const auto & tf : msg->transforms)
        if (tf.child_frame_id == transform_.child_frame_id) sources_[id] = tf;
    };
    static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(100).transient_local(), callback);
    dynamic_sub_ = create_subscription<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SensorDataQoS(), callback);
    quiet_since_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(200ms, [this] { check(); });
  }
  ~CameraTF() override { withdraw(); if (lease_ >= 0) close(lease_); }
  bool failed() const { return failed_; }

 private:
  static std::string gid(const uint8_t * data) {
    return std::string(reinterpret_cast<const char *>(data), RMW_GID_STORAGE_SIZE);
  }
  bool matches(const geometry_msgs::msg::TransformStamped & tf) {
    if (tf.header.frame_id != transform_.header.frame_id) return false;
    const auto & a = tf.transform; const auto & b = transform_.transform;
    for (double v : {a.translation.x, a.translation.y, a.translation.z,
        a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w})
      if (!std::isfinite(v)) return false;
    if (std::abs(a.translation.x-b.translation.x) > 1e-6 ||
        std::abs(a.translation.y-b.translation.y) > 1e-6 ||
        std::abs(a.translation.z-b.translation.z) > 1e-6) return false;
    for (double sign : {1., -1.})
      if (std::abs(a.rotation.x-sign*b.rotation.x) < 1e-6 &&
          std::abs(a.rotation.y-sign*b.rotation.y) < 1e-6 &&
          std::abs(a.rotation.z-sign*b.rotation.z) < 1e-6 &&
          std::abs(a.rotation.w-sign*b.rotation.w) < 1e-6) return true;
    return false;
  }
  void withdraw() { publisher_.reset(); if (lease_ >= 0) flock(lease_, LOCK_UN); }
  void check() {
    std::set<std::string> live;
    for (const auto & topic : {"/tf_static", "/tf"})
      for (const auto & p : get_publishers_info_by_topic(topic)) live.insert(gid(p.endpoint_gid().data()));
    for (auto it = sources_.begin(); it != sources_.end(); )
      if (!live.count(it->first)) it = sources_.erase(it); else ++it;
    if (!sources_.empty()) {
      withdraw(); quiet_since_ = std::chrono::steady_clock::now();
      if (sources_.size() != 1 || !matches(sources_.begin()->second)) {
        failed_ = true;
        RCLCPP_ERROR(get_logger(), "duplicate or inconsistent camera mount TF sources");
        rclcpp::shutdown();
      } else if (state_ != "reusing") {
        state_ = "reusing";
        RCLCPP_INFO(get_logger(), "Reusing existing base_link -> camera_link TF source");
      }
      return;
    }
    if (state_ == "reusing") { state_ = "waiting"; quiet_since_ = std::chrono::steady_clock::now(); }
    if (publisher_ || std::chrono::steady_clock::now() - quiet_since_ < 2s) return;
    if (flock(lease_, LOCK_EX | LOCK_NB) != 0) return;
    publisher_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).transient_local());
    owned_.insert(gid(publisher_->get_gid().data));
    transform_.header.stamp = now();
    tf2_msgs::msg::TFMessage msg; msg.transforms.push_back(transform_); publisher_->publish(msg);
    state_ = "publishing";
    RCLCPP_INFO(get_logger(), "Publishing configured base_link -> camera_link TF (single local owner)");
  }
  int lease_{-1}; bool failed_{false}; std::string state_{"waiting"};
  geometry_msgs::msg::TransformStamped transform_;
  std::chrono::steady_clock::time_point quiet_since_;
  std::map<std::string, geometry_msgs::msg::TransformStamped> sources_;
  std::set<std::string> owned_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_, dynamic_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<CameraTF>(); rclcpp::spin(node);
    status = node->failed() ? 1 : 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("go2_camera_tf"), "%s", e.what()); status = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return status;
}
