#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace vlm_nav
{

enum class BridgeState { DISARMED, ARMED, FAULT };

struct BridgeLimits
{
  double max_vx{0.40};
  double max_vyaw{0.50};
  double cmd_timeout{0.5};
  double vy_epsilon{0.0001};
  double endpoint_loss_debounce{1.0};
};

struct SafeCommand
{
  double vx{0.0};
  double vyaw{0.0};
};

struct CommandResult
{
  bool accepted{false};
  std::string error;
  SafeCommand command;
};

class EndpointDebouncer
{
public:
  explicit EndpointDebouncer(double debounce_seconds)
  : debounce_seconds_(std::max(0.0, debounce_seconds))
  {
  }

  bool update(bool endpoint_present, double now)
  {
    if (!std::isfinite(now)) {
      return false;
    }
    if (endpoint_present) {
      seen_healthy_ = true;
      missing_since_.reset();
      return true;
    }
    if (!seen_healthy_) {
      return false;
    }
    if (!missing_since_.has_value()) {
      missing_since_ = now;
    }
    return now - *missing_since_ <= debounce_seconds_;
  }

private:
  double debounce_seconds_;
  bool seen_healthy_{false};
  std::optional<double> missing_since_;
};

class Go2BridgeStateMachine
{
public:
  explicit Go2BridgeStateMachine(BridgeLimits limits)
  : limits_(limits)
  {
  }

  BridgeState state() const { return state_; }

  const std::string & fault_reason() const { return fault_reason_; }

  std::optional<SafeCommand> active_command() const
  {
    if (state_ != BridgeState::ARMED || !motion_active_ || !last_valid_cmd_time_.has_value()) {
      return std::nullopt;
    }
    return command_;
  }

  double last_valid_cmd_age(double now) const
  {
    if (!last_valid_cmd_time_.has_value()) {
      return INFINITY;
    }
    return std::max(0.0, now - *last_valid_cmd_time_);
  }

  CommandResult handle_command(
    double vx, double vy, double vyaw, double now)
  {
    return handle_twist(vx, vy, 0.0, 0.0, 0.0, vyaw, now);
  }

  CommandResult handle_twist(
    double vx, double vy, double vz,
    double roll_rate, double pitch_rate, double yaw_rate, double now)
  {
    if (state_ == BridgeState::FAULT) {
      return {false, "FAULT must be explicitly reset before accepting commands", {}};
    }
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) ||
      !std::isfinite(roll_rate) || !std::isfinite(pitch_rate) ||
      !std::isfinite(yaw_rate) || !std::isfinite(now))
    {
      enter_fault("cmd_vel contains NaN/Inf");
      return {false, fault_reason_, {}};
    }
    if (std::abs(vy) > limits_.vy_epsilon ||
      std::abs(vz) > limits_.vy_epsilon ||
      std::abs(roll_rate) > limits_.vy_epsilon ||
      std::abs(pitch_rate) > limits_.vy_epsilon)
    {
      enter_fault("cmd_vel contains an unsupported lateral/vertical/roll/pitch component");
      return {false, fault_reason_, {}};
    }

    if (state_ == BridgeState::DISARMED) {
      if (vx != 0.0 || yaw_rate != 0.0) {
        return {false, "nonzero cmd_vel is forbidden while DISARMED", {}};
      }
      return {true, "", {}};
    }

    command_.vx = std::clamp(vx, -limits_.max_vx, limits_.max_vx);
    command_.vyaw = std::clamp(yaw_rate, -limits_.max_vyaw, limits_.max_vyaw);
    const bool zero = command_.vx == 0.0 && command_.vyaw == 0.0;
    if (zero && state_ == BridgeState::ARMED) {
      pending_stop_edge_ = true;
    }
    motion_active_ = !zero;
    last_valid_cmd_time_ = now;
    return {true, "", command_};
  }

  bool arm(double now, std::string * reason = nullptr)
  {
    (void)now;
    if (state_ == BridgeState::FAULT) {
      set_reason(reason, "FAULT must be explicitly reset before ARM");
      return false;
    }
    if (state_ == BridgeState::ARMED) {
      return true;
    }
    clear_motion_input();
    state_ = BridgeState::ARMED;
    return true;
  }

  void disarm()
  {
    if (state_ == BridgeState::ARMED) {
      pending_stop_edge_ = true;
    }
    state_ = BridgeState::DISARMED;
    clear_motion_input();
    fault_reason_.clear();
  }

  bool reset_fault()
  {
    if (state_ != BridgeState::FAULT) {
      return false;
    }
    state_ = BridgeState::DISARMED;
    clear_motion_input();
    fault_reason_.clear();
    return true;
  }

  void force_fault(const std::string & reason)
  {
    enter_fault(reason);
  }

  void tick(double now, bool cmd_endpoint_present)
  {
    if (state_ != BridgeState::ARMED) {
      endpoint_missing_since_.reset();
      return;
    }

    if (motion_active_ && last_valid_cmd_age(now) > limits_.cmd_timeout) {
      enter_fault("last_valid_cmd_age exceeds cmd_timeout");
      return;
    }

    if (cmd_endpoint_present) {
      endpoint_missing_since_.reset();
      return;
    }
    if (!endpoint_missing_since_.has_value()) {
      endpoint_missing_since_ = now;
      return;
    }
    if (now - *endpoint_missing_since_ > limits_.endpoint_loss_debounce) {
      enter_fault("cmd_vel DDS endpoint absent beyond debounce");
    }
  }

  bool consume_stop_edge()
  {
    const bool value = pending_stop_edge_;
    pending_stop_edge_ = false;
    return value;
  }

private:
  static void set_reason(std::string * destination, const std::string & value)
  {
    if (destination != nullptr) {
      *destination = value;
    }
  }

  void clear_motion_input()
  {
    command_ = {};
    motion_active_ = false;
    last_valid_cmd_time_.reset();
    endpoint_missing_since_.reset();
  }

  void enter_fault(const std::string & reason)
  {
    if (state_ == BridgeState::ARMED) {
      pending_stop_edge_ = true;
    }
    state_ = BridgeState::FAULT;
    fault_reason_ = reason;
    clear_motion_input();
  }

  BridgeLimits limits_;
  BridgeState state_{BridgeState::DISARMED};
  SafeCommand command_;
  std::optional<double> last_valid_cmd_time_;
  std::optional<double> endpoint_missing_since_;
  bool pending_stop_edge_{false};
  bool motion_active_{false};
  std::string fault_reason_;
};

inline const char * to_string(BridgeState state)
{
  switch (state) {
    case BridgeState::DISARMED:
      return "DISARMED";
    case BridgeState::ARMED:
      return "ARMED";
    case BridgeState::FAULT:
      return "FAULT";
  }
  return "FAULT";
}

}  // namespace vlm_nav
