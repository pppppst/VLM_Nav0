#include "vlm_nav/go2_bridge_state.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_api/msg/request.hpp"

namespace vlm_nav
{

class TwistToGo2SportBridge : public rclcpp::Node
{
public:
  TwistToGo2SportBridge()
  : Node("twist_to_go2_sport_bridge"),
    dry_run_(declare_parameter<bool>("dry_run", true)),
    default_armed_(declare_parameter<bool>("default_armed", false)),
    require_system_ready_(declare_parameter<bool>("require_system_ready", true)),
    system_ready_timeout_(declare_parameter<double>("system_ready_timeout", 0.5)),
    limits_{
      declare_parameter<double>("max_vx", 0.20),
      declare_parameter<double>("max_vyaw", 0.40),
      declare_parameter<double>("cmd_timeout", 0.5),
      declare_parameter<double>("vy_epsilon", 0.0001),
      declare_parameter<double>("endpoint_loss_debounce", 1.0)},
    state_machine_(limits_),
    sport_endpoint_debouncer_(limits_.endpoint_loss_debounce)
  {
    const auto cmd_vel_topic = declare_parameter<std::string>(
      "cmd_vel_topic", "/cmd_vel_bridge");
    const auto sport_request_topic = declare_parameter<std::string>(
      "sport_request_topic", "/api/sport/request");
    const auto debug_request_topic = declare_parameter<std::string>(
      "debug_request_topic", "/vlm_nav/go2_sport_request_debug");
    const auto state_topic = declare_parameter<std::string>(
      "state_topic", "/vlm_nav/go2_bridge_state");
    const auto system_ready_topic = declare_parameter<std::string>(
      "system_ready_topic", "/vlm_nav/system_ready");
    const auto control_armed_topic = declare_parameter<std::string>(
      "control_armed_topic", "/vlm_nav/control_armed");
    const auto sport_interface_ready_topic = declare_parameter<std::string>(
      "sport_interface_ready_topic", "/vlm_nav/go2_sport_interface_ready");
    const auto control_rate = declare_parameter<double>("control_rate", 20.0);

    if (default_armed_) {
      throw std::invalid_argument("default_armed=true is forbidden for the Go2 bridge");
    }
    if (control_rate <= 0.0 || limits_.cmd_timeout <= 0.0 ||
      limits_.endpoint_loss_debounce < 0.0 || limits_.vy_epsilon < 0.0 ||
      system_ready_timeout_ <= 0.0)
    {
      throw std::invalid_argument("invalid Go2 bridge safety parameters");
    }

    sport_request_pub_ = create_publisher<unitree_api::msg::Request>(
      sport_request_topic, rclcpp::QoS(10).reliable());
    debug_request_pub_ = create_publisher<std_msgs::msg::String>(
      debug_request_topic, rclcpp::QoS(10).reliable());
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic, rclcpp::QoS(1).reliable().transient_local());
    control_armed_pub_ = create_publisher<std_msgs::msg::Bool>(
      control_armed_topic, rclcpp::QoS(1).reliable().transient_local());
    sport_interface_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      sport_interface_ready_topic, rclcpp::QoS(1).reliable().transient_local());
    system_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      system_ready_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &TwistToGo2SportBridge::on_system_ready, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      rclcpp::QoS(10).reliable(),
      std::bind(&TwistToGo2SportBridge::on_command, this, std::placeholders::_1));

    arm_service_ = create_service<std_srvs::srv::Trigger>(
      "~/arm",
      std::bind(
        &TwistToGo2SportBridge::on_arm, this, std::placeholders::_1,
        std::placeholders::_2));
    disarm_service_ = create_service<std_srvs::srv::Trigger>(
      "~/disarm",
      std::bind(
        &TwistToGo2SportBridge::on_disarm, this, std::placeholders::_1,
        std::placeholders::_2));
    reset_fault_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_fault",
      std::bind(
        &TwistToGo2SportBridge::on_reset_fault, this, std::placeholders::_1,
        std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate),
      std::bind(&TwistToGo2SportBridge::control_tick, this));
    publish_state();
    RCLCPP_WARN(
      get_logger(), "Go2 Sport bridge started %s and DISARMED",
      dry_run_ ? "in dry-run mode" : "with real Sport output enabled");
  }

  ~TwistToGo2SportBridge() override
  {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_machine_.state() == BridgeState::ARMED) {
        state_machine_.disarm();
        publish_pending_stop_edge();
      }
    } catch (...) {
      // Best effort only: SIGKILL, power loss, and network loss cannot be covered.
    }
  }

private:
  static constexpr std::int64_t kMoveApiId = 1008;
  static constexpr std::int64_t kStopMoveApiId = 1003;

  double steady_seconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void on_command(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = state_machine_.state();
    const auto result = state_machine_.handle_twist(
      message->linear.x, message->linear.y, message->linear.z,
      message->angular.x, message->angular.y, message->angular.z,
      steady_seconds());
    if (!result.accepted) {
      RCLCPP_ERROR(get_logger(), "rejected cmd_vel: %s", result.error.c_str());
    }
    handle_transition_edges(before);
  }

  void on_arm(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (require_system_ready_ && !system_ready_contract_valid(steady_seconds())) {
      response->success = false;
      response->message = "SYSTEM_READY gate is false, stale, or has non-unique ownership";
      return;
    }
    std::string reason;
    response->success = state_machine_.arm(steady_seconds(), &reason);
    response->message = response->success ? "ARMED" : reason;
    publish_state();
  }

  void on_system_ready(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = state_machine_.state();
    system_ready_ = message->data;
    last_system_ready_received_ = steady_seconds();
    if (require_system_ready_ && !system_ready_ && before == BridgeState::ARMED) {
      state_machine_.force_fault("SYSTEM_READY lost while ARMED");
      handle_transition_edges(before);
    }
  }

  void on_disarm(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = state_machine_.state();
    state_machine_.disarm();
    handle_transition_edges(before);
    response->success = true;
    response->message = "DISARMED";
  }

  void on_reset_fault(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = state_machine_.reset_fault();
    response->message = response->success ? "FAULT reset; bridge is DISARMED" :
      "bridge is not in FAULT";
    publish_state();
  }

  void control_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = state_machine_.state();
    const auto now = steady_seconds();
    const bool raw_sport_endpoint = sport_request_pub_->get_subscription_count() > 0;
    const bool sport_interface_ready =
      sport_endpoint_debouncer_.update(raw_sport_endpoint, now);
    publish_sport_interface_ready(sport_interface_ready);
    if (state_machine_.state() == BridgeState::ARMED) {
      if (require_system_ready_ && !system_ready_contract_valid(now)) {
        state_machine_.force_fault("SYSTEM_READY became stale or ownership is not unique");
      }
      if (!sport_interface_ready) {
        state_machine_.force_fault("Sport interface endpoint absent beyond debounce");
      }
    }
    state_machine_.tick(now, cmd_sub_->get_publisher_count() > 0);
    handle_transition_edges(before);
    if (const auto command = state_machine_.active_command()) {
      publish_move(command->vx, command->vyaw);
    }
    publish_state();
  }

  bool system_ready_contract_valid(double now) const
  {
    return system_ready_ && last_system_ready_received_.has_value() &&
      now - *last_system_ready_received_ <= system_ready_timeout_ &&
      system_ready_sub_->get_publisher_count() == 1;
  }

  void handle_transition_edges(BridgeState before)
  {
    publish_pending_stop_edge();
    if (before != state_machine_.state()) {
      if (state_machine_.state() == BridgeState::FAULT) {
        RCLCPP_ERROR(
          get_logger(), "bridge entered FAULT: %s",
          state_machine_.fault_reason().c_str());
      }
      publish_state();
    }
  }

  void publish_pending_stop_edge()
  {
    if (!state_machine_.consume_stop_edge()) {
      return;
    }
    publish_move(0.0, 0.0);
    publish_api_request(kStopMoveApiId, "{}");
  }

  void publish_move(double vx, double vyaw)
  {
    std::ostringstream parameter;
    parameter << std::setprecision(10) << "{\"x\":" << vx <<
      ",\"y\":0.0,\"z\":" << vyaw << "}";
    publish_api_request(kMoveApiId, parameter.str());
  }

  void publish_api_request(std::int64_t api_id, const std::string & parameter)
  {
    if (dry_run_) {
      std_msgs::msg::String debug;
      debug.data = "{\"api_id\":" + std::to_string(api_id) +
        ",\"parameter\":" + quote_json(parameter) + "}";
      debug_request_pub_->publish(debug);
      return;
    }
    unitree_api::msg::Request request;
    request.header.identity.id = ++request_id_;
    request.header.identity.api_id = api_id;
    request.parameter = parameter;
    sport_request_pub_->publish(request);
  }

  static std::string quote_json(const std::string & input)
  {
    std::string output = "\"";
    for (char character : input) {
      if (character == '\\' || character == '"') {
        output.push_back('\\');
      }
      output.push_back(character);
    }
    output.push_back('"');
    return output;
  }

  void publish_state()
  {
    std_msgs::msg::String message;
    message.data = to_string(state_machine_.state());
    if (state_machine_.state() == BridgeState::FAULT) {
      message.data += ": " + state_machine_.fault_reason();
    }
    state_pub_->publish(message);
    std_msgs::msg::Bool armed;
    armed.data = state_machine_.state() == BridgeState::ARMED;
    control_armed_pub_->publish(armed);
  }

  void publish_sport_interface_ready(bool ready)
  {
    std_msgs::msg::Bool message;
    message.data = ready;
    sport_interface_ready_pub_->publish(message);
  }

  bool dry_run_;
  bool default_armed_;
  bool require_system_ready_;
  double system_ready_timeout_;
  bool system_ready_{false};
  BridgeLimits limits_;
  Go2BridgeStateMachine state_machine_;
  EndpointDebouncer sport_endpoint_debouncer_;
  std::mutex mutex_;
  std::int64_t request_id_{0};
  std::optional<double> last_system_ready_received_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_armed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sport_interface_ready_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr system_ready_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vlm_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vlm_nav::TwistToGo2SportBridge>());
  rclcpp::shutdown();
  return 0;
}
