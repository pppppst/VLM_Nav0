#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <message_filters/subscriber.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "vlm_nav/obstacle_cloud_filter.hpp"
#include "vlm_nav/temporal_obstacle_filter.hpp"

namespace vlm_nav
{

class ObstacleCloudFilterNode : public rclcpp::Node
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using SteadyClock = std::chrono::steady_clock;

  ObstacleCloudFilterNode()
  : Node("obstacle_cloud_filter"),
    last_diagnostic_at_(SteadyClock::now())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    output_topic_ = declare_parameter<std::string>("output_topic", "/vlm_nav/obstacle_cloud");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    tf_queue_size_ = declare_parameter<int>("tf_queue_size", 20);
    parameters_.min_height = declare_parameter<double>("min_height", 0.05);
    parameters_.max_height = declare_parameter<double>("max_height", 1.50);
    parameters_.self_crop_min_x = declare_parameter<double>("self_crop_min_x", -0.36);
    parameters_.self_crop_max_x = declare_parameter<double>("self_crop_max_x", 0.36);
    parameters_.self_crop_min_y = declare_parameter<double>("self_crop_min_y", -0.25);
    parameters_.self_crop_max_y = declare_parameter<double>("self_crop_max_y", 0.25);
    parameters_.self_crop_min_z = declare_parameter<double>("self_crop_min_z", 0.05);
    parameters_.self_crop_max_z = declare_parameter<double>("self_crop_max_z", 1.50);
    parameters_.voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.05);
    rcl_interfaces::msg::ParameterDescriptor intensity_descriptor;
    intensity_descriptor.read_only = true;
    intensity_descriptor.description = "Experimental post-voxel filter; restart to switch A/B";
    parameters_.intensity_filter_enabled = declare_parameter<bool>(
      "intensity_filter_enabled", false, intensity_descriptor);
    parameters_.intensity_threshold = declare_parameter<double>(
      "intensity_threshold", 3.0, intensity_descriptor);
    validate_configuration();

    publisher_ = create_publisher<Cloud>(output_topic_, rclcpp::SensorDataQoS());
    rcl_interfaces::msg::ParameterDescriptor temporal_descriptor;
    temporal_descriptor.read_only = true;
    const bool temporal_enabled = declare_parameter<bool>(
      "temporal_confirmation_enabled", false, temporal_descriptor);
    const double temporal_period = declare_parameter<double>(
      "temporal_confirmation_period", 0.8, temporal_descriptor);
    const int temporal_cycles = declare_parameter<int>(
      "temporal_confirmation_cycles", 3, temporal_descriptor);
    if (temporal_enabled) {
      if (temporal_cycles <= 0) {
        throw std::invalid_argument("temporal_confirmation_cycles must be positive");
      }
      temporal_filter_ = std::make_unique<TemporalObstacleFilter>(temporal_period, temporal_cycles);
      statistics_.confirmation_status = "waiting_for_map";
      confirmed_publisher_ = create_publisher<Cloud>(
        output_topic_ + "_confirmed", rclcpp::SensorDataQoS());
      map_subscriber_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local().reliable(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
          std::lock_guard<std::recursive_mutex> lock(temporal_mutex_);
          try {
            if (temporal_filter_->set_map(*map)) {
              map_cloud_filter_->clear();
              map_cloud_filter_->setTargetFrame(temporal_filter_->frame());
            }
          } catch (const std::exception & error) {
            map_cloud_filter_->clear();
            RCLCPP_ERROR(get_logger(), "Cannot confirm obstacles: %s", error.what());
          }
        });
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    if (temporal_filter_) {
      map_cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<Cloud>>(
        *tf_buffer_, "", static_cast<uint32_t>(tf_queue_size_), get_node_logging_interface(),
        get_node_clock_interface(), std::chrono::nanoseconds(std::llround(temporal_period * 1.e9)));
      map_cloud_filter_->registerCallback(
        std::bind(&ObstacleCloudFilterNode::on_map_cloud, this, std::placeholders::_1));
    }

    cloud_subscriber_ = std::make_shared<message_filters::Subscriber<Cloud>>(
      this, input_topic_, rmw_qos_profile_sensor_data);
    // Humble's MessageFilter waits for the transform at cloud.header.stamp.
    // Its time tolerance remains the default zero: no stamp + tolerance TF is requested.
    cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<Cloud>>(
      *cloud_subscriber_, *tf_buffer_, target_frame_,
      static_cast<std::uint32_t>(tf_queue_size_), get_node_logging_interface(),
      get_node_clock_interface());
    cloud_filter_->setTolerance(rclcpp::Duration::from_seconds(0.0));
    cloud_filter_->registerCallback(
      std::bind(&ObstacleCloudFilterNode::on_cloud, this, std::placeholders::_1));

    updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("obstacle_cloud_filter");
    updater_->add(
      "Obstacle cloud preprocessing", this,
      &ObstacleCloudFilterNode::produce_diagnostics);

    RCLCPP_INFO(
      get_logger(), "Filtering %s into %s in frame %s",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  struct RuntimeStatistics
  {
    FilterStatistics points;
    std::uint64_t processed_count{0};
    double processing_latency_ms{0.0};
    std::optional<SteadyClock::time_point> last_success;
    std::optional<SteadyClock::time_point> last_confirmation;
    std::size_t confirmed_points{0};
    std::string confirmation_status{"disabled"};
  };

  void validate_configuration()
  {
    if (input_topic_.empty() || output_topic_.empty() || target_frame_.empty()) {
      throw std::invalid_argument("input_topic, output_topic, and target_frame must not be empty");
    }
    if (tf_queue_size_ <= 0) {
      throw std::invalid_argument("tf_queue_size must be greater than zero");
    }
    validate_filter_parameters(parameters_);
  }

  void on_cloud(const Cloud::ConstSharedPtr & cloud)
  {
    const auto started = SteadyClock::now();
    Cloud transformed;
    try {
      if (!pcl_ros::transformPointCloud(target_frame_, *cloud, transformed, *tf_buffer_)) {
        RCLCPP_ERROR(
          get_logger(), "TF became unavailable while transforming cloud at stamp %d.%09u",
          cloud->header.stamp.sec, cloud->header.stamp.nanosec);
        return;
      }
      transformed.header.stamp = cloud->header.stamp;
      transformed.header.frame_id = target_frame_;

      FilterStatistics points;
      auto output = filter_base_link_cloud(transformed, parameters_, points);
      output.header.stamp = cloud->header.stamp;
      output.header.frame_id = target_frame_;
      publisher_->publish(output);

      if (temporal_filter_) {
        std::lock_guard<std::recursive_mutex> lock(temporal_mutex_);
        const auto & frame = temporal_filter_->frame();
        if (frame.empty() || !tf_buffer_->canTransform(
            frame, output.header.frame_id, rclcpp::Time(output.header.stamp)))
        {
          auto empty = output;
          empty.data.clear();
          empty.width = empty.row_step = 0;
          empty.height = 1;
          confirmed_publisher_->publish(empty);
          std::lock_guard<std::mutex> stats_lock(statistics_mutex_);
          statistics_.confirmation_status = frame.empty() ? "waiting_for_map" : "waiting_for_map_tf";
          statistics_.confirmed_points = 0;
        }
        // A bounded TF queue must not block publication of subsequent clearing clouds.
        if (!frame.empty()) {map_cloud_filter_->add(std::make_shared<Cloud>(std::move(output)));}
      }

      const auto completed = SteadyClock::now();
      const auto latency = std::chrono::duration<double, std::milli>(completed - started).count();
      std::lock_guard<std::mutex> lock(statistics_mutex_);
      statistics_.points = points;
      statistics_.processing_latency_ms = latency;
      statistics_.last_success = completed;
      ++statistics_.processed_count;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Obstacle cloud processing failed: %s", error.what());
    }
  }

  void on_map_cloud(const Cloud::ConstSharedPtr & cloud)
  {
    std::lock_guard<std::recursive_mutex> lock(temporal_mutex_);
    try {
      const auto transform = tf_buffer_->lookupTransform(
        temporal_filter_->frame(), cloud->header.frame_id, rclcpp::Time(cloud->header.stamp));
      Cloud map_cloud;
      pcl_ros::transformPointCloud(temporal_filter_->frame(), transform, *cloud, map_cloud);
      auto confirmed = temporal_filter_->filter(map_cloud);
      confirmed_publisher_->publish(confirmed);
      std::lock_guard<std::mutex> stats_lock(statistics_mutex_);
      statistics_.confirmed_points = confirmed.width;
      statistics_.confirmation_status = "active";
      statistics_.last_confirmation = SteadyClock::now();
    } catch (const std::exception & error) {
      temporal_filter_->reset();
      std::lock_guard<std::mutex> stats_lock(statistics_mutex_);
      statistics_.confirmation_status = "invalid_map_cloud";
      statistics_.last_confirmation.reset();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Obstacle confirmation withheld: %s", error.what());
    }
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const auto now = SteadyClock::now();
    RuntimeStatistics snapshot;
    double frequency = 0.0;
    {
      std::lock_guard<std::mutex> lock(statistics_mutex_);
      snapshot = statistics_;
      const auto elapsed = std::chrono::duration<double>(now - last_diagnostic_at_).count();
      if (elapsed > 0.0) {
        frequency = static_cast<double>(
          snapshot.processed_count - last_diagnostic_count_) / elapsed;
      }
      last_diagnostic_count_ = snapshot.processed_count;
      last_diagnostic_at_ = now;
    }

    const double last_success_age = snapshot.last_success.has_value() ?
      std::chrono::duration<double>(now - *snapshot.last_success).count() :
      std::numeric_limits<double>::infinity();
    const double confirmation_age = snapshot.last_confirmation.has_value() ?
      std::chrono::duration<double>(now - *snapshot.last_confirmation).count() :
      std::numeric_limits<double>::infinity();
    if (!snapshot.last_success.has_value()) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No cloud has been processed");
    } else if (last_success_age > 2.5) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Obstacle cloud stream is stale");
    } else if (temporal_filter_ && confirmation_age > 0.8) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Temporal obstacle confirmation unavailable: " + snapshot.confirmation_status);
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Obstacle cloud processing active");
    }
    status.add("processed_frequency_hz", frequency);
    status.add("input_points", snapshot.points.input_points);
    status.add("after_height", snapshot.points.after_height);
    status.add("after_self_crop", snapshot.points.after_self_crop);
    status.add("after_voxel", snapshot.points.after_voxel);
    status.add("output_points", snapshot.points.output_points);
    status.add("confirmed_points", snapshot.confirmed_points);
    status.add("confirmation_status", snapshot.confirmation_status);
    status.add("last_confirmation_age_s", confirmation_age);
    status.add("intensity_filter_enabled", parameters_.intensity_filter_enabled);
    status.add("intensity_threshold", parameters_.intensity_threshold);
    status.add("intensity_removed", snapshot.points.after_voxel - snapshot.points.output_points);
    status.add("processing_latency_ms", snapshot.processing_latency_ms);
    status.add("last_success_age_s", last_success_age);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  int tf_queue_size_{20};
  FilterParameters parameters_;

  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Publisher<Cloud>::SharedPtr confirmed_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  std::unique_ptr<TemporalObstacleFilter> temporal_filter_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<message_filters::Subscriber<Cloud>> cloud_subscriber_;
  std::shared_ptr<tf2_ros::MessageFilter<Cloud>> cloud_filter_;
  std::shared_ptr<tf2_ros::MessageFilter<Cloud>> map_cloud_filter_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  std::mutex statistics_mutex_;
  std::recursive_mutex temporal_mutex_;
  RuntimeStatistics statistics_;
  SteadyClock::time_point last_diagnostic_at_;
  std::uint64_t last_diagnostic_count_{0};
};

}  // namespace vlm_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vlm_nav::ObstacleCloudFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_cloud_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
