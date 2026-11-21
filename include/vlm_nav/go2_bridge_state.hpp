#pragma once

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <optional>
#include <string>

namespace vlm_nav
{

enum class BridgeState { DISARMED, ARMED, FAULT };
enum class BridgeFaultCode {
  NONE, SYSTEM_READY_FALSE, SYSTEM_READY_STALE, SPORT_ENDPOINT_LOST,
  CMD_ENDPOINT_LOST, CMD_TIMEOUT, OWNERSHIP_CONFLICT, INVALID_COMMAND,
  FOREIGN_CONTROL, UNKNOWN
};
enum class RecoveryPhase : std::uint8_t { PREPARE = 1, RESUME = 2, CANCEL = 3 };

inline bool is_recoverable(BridgeFaultCode code)
{
  return code == BridgeFaultCode::SYSTEM_READY_FALSE ||
    code == BridgeFaultCode::SYSTEM_READY_STALE ||
    code == BridgeFaultCode::SPORT_ENDPOINT_LOST ||
    code == BridgeFaultCode::CMD_ENDPOINT_LOST || code == BridgeFaultCode::CMD_TIMEOUT;
}

inline const char * to_string(BridgeFaultCode code)
{
  switch (code) {
    case BridgeFaultCode::NONE: return "NONE";
    case BridgeFaultCode::SYSTEM_READY_FALSE: return "SYSTEM_READY_FALSE";
    case BridgeFaultCode::SYSTEM_READY_STALE: return "SYSTEM_READY_STALE";
    case BridgeFaultCode::SPORT_ENDPOINT_LOST: return "SPORT_ENDPOINT_LOST";
    case BridgeFaultCode::CMD_ENDPOINT_LOST: return "CMD_ENDPOINT_LOST";
    case BridgeFaultCode::CMD_TIMEOUT: return "CMD_TIMEOUT";
    case BridgeFaultCode::OWNERSHIP_CONFLICT: return "OWNERSHIP_CONFLICT";
    case BridgeFaultCode::INVALID_COMMAND: return "INVALID_COMMAND";
    case BridgeFaultCode::FOREIGN_CONTROL: return "FOREIGN_CONTROL";
    case BridgeFaultCode::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

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
  BridgeFaultCode fault_code() const { return fault_code_; }
  const std::string & instance_id() const { return instance_id_; }
  std::uint64_t epoch() const { return epoch_; }
  bool recovery_pending() const { return recovery_pending_; }
  bool recovery_allowed() const
  {
    return explicitly_armed_ && is_recoverable(fault_code_) &&
      (state_ == BridgeState::FAULT || recovery_pending_);
  }

  void revoke_recovery()
  {
    resume_source_epoch_.reset();
    ++epoch_;
    explicitly_armed_ = false;
    recovery_pending_ = false;
  }

  bool recover(
    const std::string & instance_id, std::uint64_t expected_epoch,
    RecoveryPhase phase, double now, const std::string & arm_blocker = "",
    std::string * reason = nullptr)
  {
    (void)now;
    const bool cancel_resumed_epoch = phase == RecoveryPhase::CANCEL &&
      state_ == BridgeState::ARMED && resume_source_epoch_.has_value() &&
      expected_epoch == *resume_source_epoch_ && epoch_ == expected_epoch + 1;
    if (instance_id != instance_id_ || (expected_epoch != epoch_ && !cancel_resumed_epoch)) {
      set_reason(reason, "bridge instance or recovery epoch changed");
      return false;
    }
    if (phase == RecoveryPhase::CANCEL) {
      // A cancellation racing RESUME stops only that automatic ARM, never a later manual ARM.
      if (state_ == BridgeState::ARMED && resume_source_epoch_.has_value()) {
        disarm();
      } else {
        revoke_recovery();
      }
      set_reason(reason, "recovery cancelled");
      return true;
    }
    if (phase == RecoveryPhase::PREPARE && state_ == BridgeState::FAULT &&
      recovery_allowed())
    {
      state_ = BridgeState::DISARMED;
      clear_motion_input();
      recovery_pending_ = true;
      set_reason(reason, "recovery prepared; bridge is DISARMED");
      return true;
    }
    if (phase == RecoveryPhase::RESUME && state_ == BridgeState::DISARMED &&
      recovery_pending_ && recovery_allowed())
    {
      if (!arm_blocker.empty()) {
        set_reason(reason, arm_blocker);
        return false;
      }
      resume_source_epoch_ = epoch_;
      ++epoch_;
      recovery_pending_ = false;
      activate();
      set_reason(reason, "ARMED");
      return true;
    }
    set_reason(reason, "recovery phase is not eligible in the current state");
    return false;
  }

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
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz) ||
      !std::isfinite(roll_rate) || !std::isfinite(pitch_rate) ||
      !std::isfinite(yaw_rate) || !std::isfinite(now))
    {
      enter_fault(BridgeFaultCode::INVALID_COMMAND, "cmd_vel contains NaN/Inf");
      return {false, fault_reason_, {}};
    }
    if (std::abs(vy) > limits_.vy_epsilon ||
      std::abs(vz) > limits_.vy_epsilon ||
      std::abs(roll_rate) > limits_.vy_epsilon ||
      std::abs(pitch_rate) > limits_.vy_epsilon)
    {
      enter_fault(BridgeFaultCode::INVALID_COMMAND,
        "cmd_vel contains an unsupported lateral/vertical/roll/pitch component");
      return {false, fault_reason_, {}};
    }

    if (state_ == BridgeState::FAULT) {
      return {false, "FAULT must be explicitly reset before accepting commands", {}};
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
    revoke_recovery();
    if (state_ == BridgeState::FAULT) {
      set_reason(reason, "FAULT must be explicitly reset before ARM");
      return false;
    }
    explicitly_armed_ = true;
    if (state_ != BridgeState::ARMED) {
      activate();
    }
    return true;
  }

  void disarm()
  {
    revoke_recovery();
    if (state_ == BridgeState::ARMED) {
      pending_stop_edge_ = true;
    }
    state_ = BridgeState::DISARMED;
    clear_motion_input();
    fault_reason_.clear();
    fault_code_ = BridgeFaultCode::NONE;
  }

  void yield_to_external_control()
  {
    const bool recoverable_fault = state_ == BridgeState::FAULT && is_recoverable(fault_code_);
    revoke_recovery();
    if (state_ != BridgeState::FAULT || recoverable_fault) {
      fault_code_ = BridgeFaultCode::FOREIGN_CONTROL;
    }
    if (state_ != BridgeState::ARMED) {
      return;  // Manual traffic must not clear an unrelated fault.
    }
    state_ = BridgeState::DISARMED;
    clear_motion_input();
    pending_stop_edge_ = false;  // The external controller already owns the Sport stream.
  }

  bool reset_fault()
  {
    revoke_recovery();
    if (state_ != BridgeState::FAULT) {
      return false;
    }
    state_ = BridgeState::DISARMED;
    clear_motion_input();
    fault_reason_.clear();
    fault_code_ = BridgeFaultCode::NONE;
    return true;
  }

  void force_fault(const std::string & reason)
  {
    enter_fault(BridgeFaultCode::UNKNOWN, reason);
  }

  void force_fault(BridgeFaultCode code, const std::string & reason)
  {
    enter_fault(code, reason);
  }

  void tick(double now, bool cmd_endpoint_present)
  {
    if (state_ != BridgeState::ARMED) {
      endpoint_missing_since_.reset();
      return;
    }

    if (motion_active_ && last_valid_cmd_age(now) > limits_.cmd_timeout) {
      enter_fault(BridgeFaultCode::CMD_TIMEOUT, "last_valid_cmd_age exceeds cmd_timeout");
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
      enter_fault(BridgeFaultCode::CMD_ENDPOINT_LOST, "cmd_vel DDS endpoint absent beyond debounce");
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

  static std::string new_instance_id()
  {
    std::random_device random;
    std::ostringstream identity;
    identity << std::hex << std::chrono::steady_clock::now().time_since_epoch().count();
    for (int index = 0; index < 4; ++index) {
      identity << '-' << std::setw(8) << std::setfill('0') << random();
    }
    return identity.str();
  }

  void activate()
  {
    clear_motion_input();
    fault_reason_.clear();
    fault_code_ = BridgeFaultCode::NONE;
    state_ = BridgeState::ARMED;
  }

  void enter_fault(BridgeFaultCode code, const std::string & reason)
  {
    if (code == BridgeFaultCode::NONE) {
      code = BridgeFaultCode::UNKNOWN;
    }
    // Preserve the first hard cause; later soft symptoms cannot restore eligibility.
    if (state_ == BridgeState::FAULT &&
      (!is_recoverable(fault_code_) || is_recoverable(code)))
    {
      return;
    }
    resume_source_epoch_.reset();
    ++epoch_;
    recovery_pending_ = false;
    if (!is_recoverable(code)) {
      explicitly_armed_ = false;
    }
    if (state_ == BridgeState::ARMED) {
      pending_stop_edge_ = true;
    }
    state_ = BridgeState::FAULT;
    fault_reason_ = reason;
    fault_code_ = code;
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
  BridgeFaultCode fault_code_{BridgeFaultCode::NONE};
  const std::string instance_id_{new_instance_id()};
  std::uint64_t epoch_{0};
  std::optional<std::uint64_t> resume_source_epoch_;
  bool explicitly_armed_{false};
  bool recovery_pending_{false};
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
