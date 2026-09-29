#include <cmath>
#include <optional>

#include "dwb_core/trajectory_generator.hpp"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "vlm_nav/componentwise_min_speed.hpp"

namespace vlm_nav
{

class ComponentwiseMinSpeedGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override
  {
    dwb_plugins::StandardTrajectoryGenerator::startNewIteration(current_velocity);
    advance();
  }

  bool hasMoreTwists() override
  {
    return next_velocity_.has_value();
  }

  nav_2d_msgs::msg::Twist2D nextTwist() override
  {
    const auto velocity = next_velocity_.value();
    advance();
    return velocity;
  }

private:
  void advance()
  {
    next_velocity_.reset();
    auto kinematics = kinematics_handler_->getKinematics();
    while (dwb_plugins::StandardTrajectoryGenerator::hasMoreTwists()) {
      auto velocity = dwb_plugins::StandardTrajectoryGenerator::nextTwist();
      if (std::hypot(velocity.x, velocity.y) < 1e-9) {
        velocity.x = 0.0;
        velocity.y = 0.0;
      }
      if (std::abs(velocity.theta) < 1e-9) {
        velocity.theta = 0.0;
      }
      if (componentwise_min_speed_valid(
          velocity, kinematics.getMinSpeedXY(), kinematics.getMinSpeedTheta()))
      {
        next_velocity_ = velocity;
        return;
      }
    }
  }

  std::optional<nav_2d_msgs::msg::Twist2D> next_velocity_;
};

}  // namespace vlm_nav

PLUGINLIB_EXPORT_CLASS(vlm_nav::ComponentwiseMinSpeedGenerator, dwb_core::TrajectoryGenerator)
