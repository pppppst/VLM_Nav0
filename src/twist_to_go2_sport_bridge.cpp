#include "vlm_nav/go2_bridge_state.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

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
    foreign_sport_quiet_period_(
      declare_parameter<double>("foreign_sport_quiet_period_s", 2.0)),
    fault_on_foreign_sport_request_(
      declare_parameter<bool>("fault_on_foreign_sport_request", true)),
    limits_{
      declare_parameter<double>("max_vx", 0.40),
      declare_parameter<double>("max_vyaw", 0.50),
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
    const auto sport_response_topic = declare_parameter<std::string>(
      "sport_response_topic", "/api/sport/response");
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
      system_ready_timeout_ <= 0.0 || foreign_sport_quiet_period_ < 0.0)
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
    sport_request_sub_ = create_subscription<unitree_api::msg::Request>(
      sport_request_topic,
      rclcpp::QoS(50).best_effort(),
      std::bind(
        &TwistToGo2SportBridge::on_sport_request, this,
        std::placeholders::_1, std::placeholders::_2));
    sport_response_sub_ = create_subscription<unitree_api::msg::Response>(
      sport_response_topic,
      rclcpp::QoS(10).best_effort(),
      [](const unitree_api::msg::Response::SharedPtr) {});
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
    foreign_observation_started_ = steady_seconds();
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
    ++cmd_rx_count_;
    const auto received = steady_seconds();
    const auto before = state_machine_.state();
    const auto result = state_machine_.handle_twist(
      message->linear.x, message->linear.y, message->linear.z,
      message->angular.x, message->angular.y, message->angular.z,
      received);
    RCLCPP_DEBUG(
      get_logger(), "cmd receive steady=%.6f vx=%.6f vy=%.6f wz=%.6f %s",
      received, message->linear.x, message->linear.y, message->angular.z,
      (message->linear.x == 0.0 && message->linear.y == 0.0 &&
      message->angular.z == 0.0) ? "zero/stop" : "nonzero");
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
    const auto now = steady_seconds();
    if (require_system_ready_ && !system_ready_contract_valid(now)) {
      response->success = false;
      response->message = "SYSTEM_READY gate is false, stale, or has non-unique ownership";
      RCLCPP_WARN(get_logger(), "ARM rejected: %s", response->message.c_str());
      return;
    }
    if (cmd_sub_->get_publisher_count() == 0) {
      response->success = false;
      response->message = "cmd_vel DDS endpoint is absent";
      RCLCPP_WARN(get_logger(), "ARM rejected: %s", response->message.c_str());
      return;
    }
    if (!sport_interface_ready_) {
      response->success = false;
      response->message = "Sport interface endpoint is not ready";
      RCLCPP_WARN(get_logger(), "ARM rejected: %s", response->message.c_str());
      return;
    }
    const double quiet_since = last_foreign_sport_request_.value_or(foreign_observation_started_);
    if (now - quiet_since < foreign_sport_quiet_period_) {
      response->success = false;
      response->message = "external Sport traffic has not been quiet for " +
        std::to_string(foreign_sport_quiet_period_) + " seconds";
      RCLCPP_WARN(get_logger(), "ARM rejected: %s", response->message.c_str());
      return;
    }
    std::string reason;
    response->success = state_machine_.arm(now, &reason);
    response->message = response->success ? "ARMED" : reason;
    if (response->success) {
      RCLCPP_WARN(get_logger(), "ARM accepted steady=%.6f", now);
    } else {
      RCLCPP_WARN(get_logger(), "ARM rejected: %s", response->message.c_str());
    }
    publish_state();
  }

  void on_sport_request(
    const unitree_api::msg::Request::SharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    bool own = false;
    const auto own_gid = sport_request_pub_->get_gid();
    const auto & source_gid = message_info.get_rmw_message_info().publisher_gid;
    const auto compare_result = rmw_compare_gids_equal(&own_gid, &source_gid, &own);
    if (compare_result == RMW_RET_OK && own) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = state_machine_.state();
    last_foreign_sport_request_ = steady_seconds();
    RCLCPP_WARN(
      get_logger(),
      "foreign Sport request gid=%s api_id=%ld request_id=%ld priority=%d noreply=%s parameter=%s",
      gid_string(source_gid).c_str(), message->header.identity.api_id,
      message->header.identity.id, message->header.policy.priority,
      message->header.policy.noreply ? "true" : "false", message->parameter.c_str());
    if (compare_result != RMW_RET_OK) {
      RCLCPP_ERROR(get_logger(), "publisher GID comparison failed; treating request as foreign");
    }
    if (fault_on_foreign_sport_request_ && before == BridgeState::ARMED) {
      state_machine_.force_fault("foreign Sport request observed while ARMED");
      handle_transition_edges(before);
    }
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
    RCLCPP_INFO(get_logger(), "explicit DISARM receive steady=%.6f", steady_seconds());
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
    RCLCPP_DEBUG(get_logger(), "watchdog check steady=%.6f", now);
    const bool raw_sport_endpoint =
      sport_request_pub_->get_subscription_count() > 0 &&
      sport_response_sub_->get_publisher_count() > 0;
    sport_interface_ready_ =
      sport_endpoint_debouncer_.update(raw_sport_endpoint, now);
    publish_sport_interface_ready(sport_interface_ready_);
    if (state_machine_.state() == BridgeState::ARMED) {
      if (require_system_ready_ && !system_ready_contract_valid(now)) {
        state_machine_.force_fault("SYSTEM_READY became stale or ownership is not unique");
      }
      if (!sport_interface_ready_) {
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
    RCLCPP_INFO(get_logger(), "StopMove publish steady=%.6f", steady_seconds());
    publish_move(0.0, 0.0);
    publish_api_request(kStopMoveApiId, "{}");
  }

  void publish_move(double vx, double vyaw)
  {
    ++sport_move_attempt_count_;
    std::ostringstream parameter;
    parameter << std::setprecision(10) << "{\"x\":" << vx <<
      ",\"y\":0.0,\"z\":" << vyaw << "}";
    const auto payload = parameter.str();
    RCLCPP_DEBUG(get_logger(), "SPORT_BUILD api_id=%ld %s", kMoveApiId, payload.c_str());
    publish_api_request(kMoveApiId, payload);
  }

  void publish_api_request(std::int64_t api_id, const std::string & parameter)
  {
    if (api_id == kStopMoveApiId) {
      ++sport_stop_attempt_count_;
    }
    RCLCPP_DEBUG(
      get_logger(), "SPORT_PUBLISH attempt api_id=%ld dry_run=%s parameter=%s",
      api_id, dry_run_ ? "true" : "false", parameter.c_str());
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
    RCLCPP_DEBUG(get_logger(), "SPORT_PUBLISH result=published api_id=%ld", api_id);
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

  static std::string gid_string(const rmw_gid_t & gid)
  {
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (std::size_t index = 0; index < RMW_GID_STORAGE_SIZE; ++index) {
      if (index != 0) {
        output << ':';
      }
      output << std::setw(2) << static_cast<unsigned>(gid.data[index]);
    }
    return output.str();
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
  double foreign_sport_quiet_period_;
  bool fault_on_foreign_sport_request_;
  bool system_ready_{false};
  bool sport_interface_ready_{false};
  BridgeLimits limits_;
  Go2BridgeStateMachine state_machine_;
  EndpointDebouncer sport_endpoint_debouncer_;
  std::mutex mutex_;
  std::int64_t request_id_{0};
  std::uint64_t cmd_rx_count_{0};
  std::uint64_t sport_move_attempt_count_{0};
  std::uint64_t sport_stop_attempt_count_{0};
  std::optional<double> last_system_ready_received_;
  double foreign_observation_started_{0.0};
  std::optional<double> last_foreign_sport_request_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_armed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sport_interface_ready_pub_;
  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr sport_request_sub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr sport_response_sub_;
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
