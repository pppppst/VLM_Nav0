#include "gtest/gtest.h"

#include "dwb_core/trajectory_generator.hpp"
#include "pluginlib/class_loader.hpp"
#include "vlm_nav/componentwise_min_speed.hpp"

TEST(ComponentwiseMinSpeed, AllowsOnlyZeroOrEffectiveComponents)
{
  nav_2d_msgs::msg::Twist2D velocity;
  EXPECT_TRUE(vlm_nav::componentwise_min_speed_valid(velocity, 0.4, 0.5));

  velocity.x = 0.4;
  EXPECT_TRUE(vlm_nav::componentwise_min_speed_valid(velocity, 0.4, 0.5));
  velocity.theta = 0.08;
  EXPECT_FALSE(vlm_nav::componentwise_min_speed_valid(velocity, 0.4, 0.5));

  velocity.x = 0.08;
  velocity.theta = -0.5;
  EXPECT_FALSE(vlm_nav::componentwise_min_speed_valid(velocity, 0.4, 0.5));
  velocity.x = 0.0;
  EXPECT_TRUE(vlm_nav::componentwise_min_speed_valid(velocity, 0.4, 0.5));
}

TEST(ComponentwiseMinSpeed, PluginIsLoadable)
{
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader(
    "dwb_core", "dwb_core::TrajectoryGenerator");
  EXPECT_NE(
    loader.createSharedInstance("vlm_nav::ComponentwiseMinSpeedGenerator"), nullptr);
}
