#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace vlm_nav
{

class LowStateImuAdapter : public rclcpp::Node
{
public:
  LowStateImuAdapter()
  : Node("lowstate_imu_adapter")
  {
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/body_imu", rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile());
    subscription_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      [this](const unitree_go::msg::LowState::SharedPtr input) {
        sensor_msgs::msg::Imu output;
        // LowState has no ROS header and the upstream tick unit is undocumented.
        const auto steady_now = std::chrono::steady_clock::now();
        if (!stamp_initialized_) {
          ros_stamp_origin_ = now();
          steady_stamp_origin_ = steady_now;
          stamp_initialized_ = true;
        }
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
          steady_now - steady_stamp_origin_).count();
        const auto stamp = ros_stamp_origin_ + rclcpp::Duration::from_nanoseconds(elapsed);
        const auto nanoseconds = stamp.nanoseconds();
        output.header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
        output.header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        output.header.frame_id = "body_imu";

        const auto & imu = input->imu_state;
        output.orientation.w = imu.quaternion[0];
        output.orientation.x = imu.quaternion[1];
        output.orientation.y = imu.quaternion[2];
        output.orientation.z = imu.quaternion[3];
        output.angular_velocity.x = imu.gyroscope[0];
        output.angular_velocity.y = imu.gyroscope[1];
        output.angular_velocity.z = imu.gyroscope[2];
        output.linear_acceleration.x = imu.accelerometer[0];
        output.linear_acceleration.y = imu.accelerometer[1];
        output.linear_acceleration.z = imu.accelerometer[2];
        publisher_->publish(output);
      });

    RCLCPP_WARN(
      get_logger(),
      "adapting /lowstate imu_state to /body_imu with host receipt timestamps");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;
  bool stamp_initialized_{false};
  rclcpp::Time ros_stamp_origin_;
  std::chrono::steady_clock::time_point steady_stamp_origin_;
};

}  // namespace vlm_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vlm_nav::LowStateImuAdapter>());
  rclcpp::shutdown();
  return 0;
}
