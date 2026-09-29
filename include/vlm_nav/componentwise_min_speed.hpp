#pragma once

#include <cmath>

#include "nav_2d_msgs/msg/twist2_d.hpp"

namespace vlm_nav
{

inline bool componentwise_min_speed_valid(
  const nav_2d_msgs::msg::Twist2D & velocity,
  double min_speed_xy,
  double min_speed_theta)
{
  const double speed_xy = std::hypot(velocity.x, velocity.y);
  return (speed_xy == 0.0 || min_speed_xy < 0.0 || speed_xy >= min_speed_xy) &&
         (velocity.theta == 0.0 || min_speed_theta < 0.0 ||
         std::abs(velocity.theta) >= min_speed_theta);
}

}  // namespace vlm_nav
