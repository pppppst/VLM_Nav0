#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "vlm_nav/go2_bridge_state.hpp"

using vlm_nav::BridgeLimits;
using vlm_nav::BridgeState;
using vlm_nav::Go2BridgeStateMachine;
using vlm_nav::EndpointDebouncer;

namespace
{
BridgeLimits limits()
{
  return BridgeLimits{0.20, 0.40, 0.5, 0.0001, 1.0};
}

void valid_then_arm(Go2BridgeStateMachine & machine, double now = 1.0)
{
  ASSERT_TRUE(machine.arm(now));
  ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.2, now).accepted);
}
}  // namespace

TEST(Go2BridgeState, StartsDisarmed)
{
  Go2BridgeStateMachine machine(limits());
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
  EXPECT_FALSE(machine.active_command().has_value());
}

TEST(Go2BridgeState, RejectsNonFiniteCommand)
{
  Go2BridgeStateMachine machine(limits());
  auto result = machine.handle_command(NAN, 0.0, 0.0, 1.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
}

TEST(Go2BridgeState, RejectsLateralVelocity)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  auto result = machine.handle_command(0.1, 0.01, 0.0, 1.1);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
  EXPECT_TRUE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, ClampsAcceptedCommand)
{
  Go2BridgeStateMachine machine(limits());
  ASSERT_TRUE(machine.arm(1.0));
  auto result = machine.handle_command(2.0, 0.0, -4.0, 1.0);
  ASSERT_TRUE(result.accepted);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.20);
  EXPECT_DOUBLE_EQ(result.command.vyaw, -0.40);
}

TEST(Go2BridgeState, AcceptsPositiveYaw)
{
  Go2BridgeStateMachine machine(limits());
  ASSERT_TRUE(machine.arm(1.0));
  auto result = machine.handle_command(0.0, 0.0, 0.2, 1.0);
  ASSERT_TRUE(result.accepted);
  EXPECT_DOUBLE_EQ(result.command.vyaw, 0.2);
}

TEST(Go2BridgeState, AcceptsNegativeYaw)
{
  Go2BridgeStateMachine machine(limits());
  ASSERT_TRUE(machine.arm(1.0));
  auto result = machine.handle_command(0.0, 0.0, -0.2, 1.0);
  ASSERT_TRUE(result.accepted);
  EXPECT_DOUBLE_EQ(result.command.vyaw, -0.2);
}

TEST(Go2BridgeState, CommandTimeoutFaultsAndStopsOnce)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.tick(1.51, true);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
  EXPECT_TRUE(machine.consume_stop_edge());
  EXPECT_FALSE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, ExplicitZeroStopsWithoutWatchdogFault)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  ASSERT_TRUE(machine.handle_command(0.0, 0.0, 0.0, 1.2).accepted);
  EXPECT_FALSE(machine.active_command().has_value());
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  machine.tick(2.0, true);
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  EXPECT_TRUE(machine.consume_stop_edge());
  EXPECT_FALSE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, LateZeroCannotClearFault)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.tick(1.51, true);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
  EXPECT_FALSE(machine.handle_command(0.0, 0.0, 0.0, 1.6).accepted);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
}

TEST(Go2BridgeState, EndpointLossUsesDebounce)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.tick(1.1, false);
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.0, 1.9).accepted);
  machine.tick(2.05, false);
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.0, 2.2).accepted);
  machine.tick(2.21, false);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
}

TEST(Go2BridgeState, FaultNeverAutomaticallyRearms)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.tick(1.51, true);
  EXPECT_FALSE(machine.handle_command(0.0, 0.0, 0.0, 1.6).accepted);
  EXPECT_FALSE(machine.arm(1.6));
  EXPECT_TRUE(machine.reset_fault());
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
}

TEST(Go2BridgeState, DisarmedRejectsAndDoesNotCacheNonzeroCommand)
{
  Go2BridgeStateMachine machine(limits());
  auto result = machine.handle_command(0.2, 0.0, 0.0, 1.0);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.error, "nonzero cmd_vel is forbidden while DISARMED");
  EXPECT_FALSE(machine.active_command().has_value());
  ASSERT_TRUE(machine.arm(2.0));
  EXPECT_FALSE(machine.active_command().has_value());
}

TEST(Go2BridgeState, ArmDoesNotRequireOrActivateCachedCommand)
{
  Go2BridgeStateMachine machine(limits());
  ASSERT_TRUE(machine.handle_command(0.0, 0.0, 0.0, 1.0).accepted);
  ASSERT_TRUE(machine.arm(2.0));
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  EXPECT_FALSE(machine.active_command().has_value());
}

TEST(Go2BridgeState, DisarmStopsOnce)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.disarm();
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
  EXPECT_TRUE(machine.consume_stop_edge());
  EXPECT_FALSE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, ExternalSafetyGateFaultsAndStopsOnce)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.force_fault("SYSTEM_READY lost");
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
  EXPECT_TRUE(machine.consume_stop_edge());
  EXPECT_FALSE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, ExternalTakeoverClearsMotionWithoutStopOrAutomaticRearm)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  ASSERT_TRUE(machine.handle_command(0.2, 0.0, 0.0, 1.1).accepted);
  machine.yield_to_external_control();
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
  EXPECT_FALSE(machine.active_command().has_value());
  EXPECT_FALSE(machine.consume_stop_edge());
  EXPECT_FALSE(machine.handle_command(0.2, 0.0, 0.0, 1.2).accepted);
  EXPECT_TRUE(machine.handle_command(0.0, 0.0, 0.0, 1.3).accepted);
  machine.tick(10.0, false);
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
  EXPECT_FALSE(machine.consume_stop_edge());
  ASSERT_TRUE(machine.arm(11.0));
  EXPECT_FALSE(machine.active_command().has_value());
}

TEST(Go2BridgeState, ExternalTakeoverDoesNotClearFault)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.force_fault("SYSTEM_READY lost");
  machine.yield_to_external_control();
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
  EXPECT_EQ(machine.fault_reason(), "SYSTEM_READY lost");
  EXPECT_TRUE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, RejectsUnsupportedTwistAxes)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  auto result = machine.handle_twist(0.1, 0.0, 0.01, 0.0, 0.0, 0.0, 1.1);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(machine.state(), BridgeState::FAULT);
  EXPECT_TRUE(machine.consume_stop_edge());
}

TEST(Go2BridgeState, SportEndpointHealthDebouncesOnlyAfterFirstHealthySample)
{
  EndpointDebouncer debouncer(1.0);
  EXPECT_FALSE(debouncer.update(false, 1.0));
  EXPECT_TRUE(debouncer.update(true, 1.1));
  EXPECT_TRUE(debouncer.update(false, 1.2));
  EXPECT_TRUE(debouncer.update(false, 2.19));
  EXPECT_FALSE(debouncer.update(false, 2.21));
  EXPECT_TRUE(debouncer.update(true, 2.3));
}

TEST(Go2BridgeState, FirstHardFaultIsNotOverwritten)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.force_fault("ownership conflict");
  machine.force_fault("later symptom");
  EXPECT_EQ(machine.fault_reason(), "ownership conflict");
}

using vlm_nav::BridgeFaultCode;
using vlm_nav::RecoveryPhase;

namespace
{
bool recover(Go2BridgeStateMachine & machine, RecoveryPhase phase,
  std::uint64_t epoch, const std::string & blocker = "")
{
  return machine.recover(machine.instance_id(), epoch, phase, 4.0, blocker);
}

void soft_fault(Go2BridgeStateMachine & machine)
{
  valid_then_arm(machine);
  machine.force_fault(BridgeFaultCode::SYSTEM_READY_FALSE, "SYSTEM_READY false");
}
}

TEST(Go2BridgeRecovery, InstanceIdentityIsUniqueAndFencesAllPhases)
{
  Go2BridgeStateMachine machine(limits()), other(limits());
  EXPECT_NE(machine.instance_id(), other.instance_id());
  soft_fault(machine);
  const auto epoch = machine.epoch();
  for (auto phase : {RecoveryPhase::PREPARE, RecoveryPhase::RESUME, RecoveryPhase::CANCEL}) {
    EXPECT_FALSE(machine.recover(other.instance_id(), epoch, phase, 3.0));
    EXPECT_EQ(machine.epoch(), epoch);
  }
}

TEST(Go2BridgeRecovery, PrepareResetsWithoutReadyAndResumeRequiresGateAndNewCommand)
{
  Go2BridgeStateMachine machine(limits());
  soft_fault(machine);
  const auto epoch = machine.epoch();
  EXPECT_TRUE(machine.recovery_allowed());
  ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch, "SYSTEM_READY false"));
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
  EXPECT_TRUE(machine.recovery_pending());
  EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch, "SYSTEM_READY false"));
  EXPECT_EQ(machine.state(), BridgeState::DISARMED);
  EXPECT_FALSE(machine.handle_command(0.1, 0.0, 0.0, 3.0).accepted);
  ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, epoch));
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  EXPECT_FALSE(machine.active_command());
  EXPECT_FALSE(machine.recovery_pending());
  EXPECT_GT(machine.epoch(), epoch);
  EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
  EXPECT_TRUE(machine.handle_command(0.1, 0.0, 0.0, 5.0).accepted);
  EXPECT_TRUE(machine.active_command());
}

TEST(Go2BridgeRecovery, NeverArmedAndHardFaultCannotPrepare)
{
  for (bool armed : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    if (armed) { valid_then_arm(machine); }
    machine.force_fault(BridgeFaultCode::SYSTEM_READY_STALE, "stale");
    EXPECT_EQ(machine.recovery_allowed(), armed);
    machine.force_fault("unknown cause");
    EXPECT_FALSE(machine.recovery_allowed());
    EXPECT_FALSE(recover(machine, RecoveryPhase::PREPARE, machine.epoch()));
  }
}

TEST(Go2BridgeRecovery, CancelInvalidatesPreparedAndNotYetPreparedRequests)
{
  for (bool prepared : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto epoch = machine.epoch();
    if (prepared) { ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch)); }
    ASSERT_TRUE(recover(machine, RecoveryPhase::CANCEL, epoch));
    EXPECT_GT(machine.epoch(), epoch);
    EXPECT_FALSE(machine.recovery_allowed());
    EXPECT_FALSE(machine.recovery_pending());
    EXPECT_FALSE(recover(machine, RecoveryPhase::PREPARE, epoch));
    EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
    EXPECT_FALSE(recover(machine, RecoveryPhase::PREPARE, machine.epoch()));
  }
}

TEST(Go2BridgeRecovery, HardFaultDuringFaultOrPreparedStateCannotBecomeSoftAgain)
{
  for (bool prepared : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto epoch = machine.epoch();
    if (prepared) { ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch)); }
    machine.force_fault(BridgeFaultCode::OWNERSHIP_CONFLICT, "duplicate owners");
    EXPECT_GT(machine.epoch(), epoch);
    const auto hard_epoch = machine.epoch();
    machine.force_fault(BridgeFaultCode::SYSTEM_READY_FALSE, "false");
    machine.force_fault(BridgeFaultCode::SPORT_ENDPOINT_LOST, "Sport lost");
    EXPECT_EQ(machine.epoch(), hard_epoch);
    EXPECT_EQ(machine.fault_code(), BridgeFaultCode::OWNERSHIP_CONFLICT);
    EXPECT_EQ(machine.fault_reason(), "duplicate owners");
    EXPECT_FALSE(recover(machine, RecoveryPhase::PREPARE, hard_epoch));
    EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
  }
}

TEST(Go2BridgeRecovery, InvalidCommandDuringSoftFaultRevokesRecovery)
{
  for (bool prepared : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto epoch = machine.epoch();
    if (prepared) { ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch)); }
    EXPECT_FALSE(machine.handle_command(NAN, 0.0, 0.0, 3.0).accepted);
    EXPECT_EQ(machine.fault_code(), BridgeFaultCode::INVALID_COMMAND);
    EXPECT_FALSE(machine.recovery_allowed());
    EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
  }
}

TEST(Go2BridgeRecovery, ExternalTakeoverDuringFaultOrPreparedStateRevokesRecovery)
{
  for (bool prepared : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto epoch = machine.epoch();
    if (prepared) { ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch)); }
    machine.yield_to_external_control();
    EXPECT_GT(machine.epoch(), epoch);
    EXPECT_FALSE(machine.recovery_allowed());
    EXPECT_FALSE(machine.recovery_pending());
    EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
  }
}

TEST(Go2BridgeRecovery, ManualArmDisarmAndResetInvalidatePendingEpoch)
{
  for (int action = 0; action < 3; ++action) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto epoch = machine.epoch();
    ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch));
    if (action == 0) { machine.arm(3.0); }
    if (action == 1) { machine.disarm(); }
    if (action == 2) { machine.reset_fault(); }
    EXPECT_GT(machine.epoch(), epoch);
    EXPECT_FALSE(machine.recovery_pending());
    EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
  }
}

TEST(Go2BridgeRecovery, ResumeRequiresPrepareAndOldEpochCannotCancelNewManualArm)
{
  Go2BridgeStateMachine machine(limits());
  soft_fault(machine);
  auto epoch = machine.epoch();
  EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, epoch));
  machine.disarm();
  machine.arm(3.0);
  EXPECT_FALSE(recover(machine, RecoveryPhase::CANCEL, epoch));
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  machine.force_fault(BridgeFaultCode::CMD_TIMEOUT, "timeout");
  EXPECT_FALSE(recover(machine, RecoveryPhase::PREPARE, epoch));
  EXPECT_TRUE(recover(machine, RecoveryPhase::PREPARE, machine.epoch()));
}

TEST(Go2BridgeRecovery, WatchdogFaultsHaveTypedSoftCodes)
{
  Go2BridgeStateMachine machine(limits());
  valid_then_arm(machine);
  machine.tick(1.51, true);
  EXPECT_EQ(machine.fault_code(), BridgeFaultCode::CMD_TIMEOUT);
  EXPECT_TRUE(machine.recovery_allowed());
  machine.disarm();
  machine.arm(2.0);
  machine.tick(2.0, false);
  machine.tick(3.1, false);
  EXPECT_EQ(machine.fault_code(), BridgeFaultCode::CMD_ENDPOINT_LOST);
  EXPECT_TRUE(machine.recovery_allowed());
}

TEST(Go2BridgeRecovery, CancelAndResumeEitherOrderLeaveDisarmedWithoutCachedMotion)
{
  for (bool resume_first : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto source_epoch = machine.epoch();
    machine.consume_stop_edge();
    ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, source_epoch));
    if (resume_first) {
      ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, source_epoch));
      ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.0, 5.0).accepted);
    }
    ASSERT_TRUE(recover(machine, RecoveryPhase::CANCEL, source_epoch));
    EXPECT_EQ(machine.state(), BridgeState::DISARMED);
    EXPECT_FALSE(machine.active_command());
    EXPECT_FALSE(machine.recovery_pending());
    EXPECT_FALSE(machine.recovery_allowed());
    EXPECT_EQ(machine.consume_stop_edge(), resume_first);
    EXPECT_FALSE(machine.consume_stop_edge());
    EXPECT_FALSE(recover(machine, RecoveryPhase::RESUME, source_epoch));
    EXPECT_FALSE(recover(machine, RecoveryPhase::CANCEL, source_epoch));
  }
}

TEST(Go2BridgeRecovery, LateCancelCannotDisarmManualArmAfterAutomaticResume)
{
  Go2BridgeStateMachine machine(limits());
  soft_fault(machine);
  const auto source_epoch = machine.epoch();
  ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, source_epoch));
  ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, source_epoch));
  const auto resumed_epoch = machine.epoch();
  ASSERT_TRUE(machine.arm(5.0));
  ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.0, 5.0).accepted);
  EXPECT_FALSE(recover(machine, RecoveryPhase::CANCEL, source_epoch));
  EXPECT_FALSE(recover(machine, RecoveryPhase::CANCEL, resumed_epoch));
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  EXPECT_TRUE(machine.active_command());
  ASSERT_TRUE(recover(machine, RecoveryPhase::CANCEL, machine.epoch()));
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  EXPECT_TRUE(machine.active_command());
}

TEST(Go2BridgeRecovery, LateCancelCannotAffectNewFaultOrItsRecovery)
{
  for (auto code : {BridgeFaultCode::CMD_TIMEOUT, BridgeFaultCode::OWNERSHIP_CONFLICT}) {
    Go2BridgeStateMachine machine(limits());
    soft_fault(machine);
    const auto source_epoch = machine.epoch();
    ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, source_epoch));
    ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, source_epoch));
    machine.force_fault(code, "new fault");
    const auto fault_epoch = machine.epoch();
    EXPECT_FALSE(recover(machine, RecoveryPhase::CANCEL, source_epoch));
    EXPECT_EQ(machine.epoch(), fault_epoch);
    EXPECT_EQ(machine.fault_code(), code);
    if (code == BridgeFaultCode::CMD_TIMEOUT) {
      ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, fault_epoch));
      ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, fault_epoch));
      EXPECT_FALSE(recover(machine, RecoveryPhase::CANCEL, source_epoch));
      EXPECT_EQ(machine.state(), BridgeState::ARMED);
      EXPECT_TRUE(recover(machine, RecoveryPhase::CANCEL, fault_epoch));
      EXPECT_EQ(machine.state(), BridgeState::DISARMED);
    }
  }
}

TEST(Go2BridgeRecovery, CurrentEpochCancelOnlyStopsAutomaticResumeNotManualArm)
{
  for (bool automatic : {false, true}) {
    Go2BridgeStateMachine machine(limits());
    if (automatic) {
      soft_fault(machine);
      const auto epoch = machine.epoch();
      ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, epoch));
      ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, epoch));
    } else {
      ASSERT_TRUE(machine.arm(4.0));
    }
    machine.consume_stop_edge();
    ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.0, 5.0).accepted);
    ASSERT_TRUE(recover(machine, RecoveryPhase::CANCEL, machine.epoch()));
    EXPECT_EQ(machine.state(), automatic ? BridgeState::DISARMED : BridgeState::ARMED);
    EXPECT_EQ(machine.active_command().has_value(), !automatic);
    EXPECT_EQ(machine.consume_stop_edge(), automatic);
  }
}

TEST(Go2BridgeRecovery, CancelOfResumedEpochStillRequiresCorrectInstance)
{
  Go2BridgeStateMachine machine(limits()), other(limits());
  soft_fault(machine);
  const auto source_epoch = machine.epoch();
  ASSERT_TRUE(recover(machine, RecoveryPhase::PREPARE, source_epoch));
  ASSERT_TRUE(recover(machine, RecoveryPhase::RESUME, source_epoch));
  EXPECT_FALSE(machine.recover(other.instance_id(), source_epoch, RecoveryPhase::CANCEL, 5.0));
  EXPECT_EQ(machine.state(), BridgeState::ARMED);
  EXPECT_TRUE(recover(machine, RecoveryPhase::CANCEL, source_epoch));
}
