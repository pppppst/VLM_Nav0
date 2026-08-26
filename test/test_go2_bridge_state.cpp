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
  ASSERT_TRUE(machine.handle_command(0.1, 0.0, 0.2, now).accepted);
  ASSERT_TRUE(machine.arm(now));
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
  auto result = machine.handle_command(2.0, 0.0, -4.0, 1.0);
  ASSERT_TRUE(result.accepted);
  EXPECT_DOUBLE_EQ(result.command.vx, 0.20);
  EXPECT_DOUBLE_EQ(result.command.vyaw, -0.40);
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
