#include "vlm_nav/temporal_obstacle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace vlm_nav
{
TemporalObstacleFilter::TemporalObstacleFilter(double period_seconds, unsigned cycles)
: cycles_(cycles)
{
  if (!std::isfinite(period_seconds) || period_seconds < 1.e-9 ||
    period_seconds >= static_cast<double>(std::numeric_limits<int64_t>::max()) / 1.e9 ||
    cycles == 0)
  {
    throw std::invalid_argument("Temporal period and cycle count must be positive and finite");
  }
  period_ns_ = std::llround(period_seconds * 1.e9);
}

void TemporalObstacleFilter::reset()
{
  period_start_.reset();
  last_stamp_.reset();
  hits_.clear();
  streaks_.clear();
}

bool TemporalObstacleFilter::set_map(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & m = map.info;
  const auto & p = m.origin.position;
  const auto & q = m.origin.orientation;
  if (map.header.frame_id.empty() || !std::isfinite(m.resolution) || m.resolution <= 0 ||
    m.width == 0 || m.height == 0 || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
    !std::isfinite(q.w) || std::abs(q.x) > 1.e-6 || std::abs(q.y) > 1.e-6 ||
    std::abs(q.z * q.z + q.w * q.w - 1.) > 1.e-6)
  {
    reset();
    frame_.clear();
    throw std::invalid_argument("Temporal confirmation requires valid planar map geometry");
  }
  if (frame_ != map.header.frame_id || map_ != m) {
    reset();
    map_ = m;
    frame_ = map.header.frame_id;
    const double yaw = 2. * std::atan2(q.z, q.w);
    cos_yaw_ = std::cos(yaw);
    sin_yaw_ = std::sin(yaw);
    return true;
  }
  return false;
}

sensor_msgs::msg::PointCloud2 TemporalObstacleFilter::filter(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (frame_.empty()) {
    throw std::logic_error("Temporal confirmation is waiting for map geometry");
  }
  if (cloud.header.frame_id != frame_ || cloud.is_bigendian || cloud.point_step == 0 ||
    uint64_t(cloud.row_step) < uint64_t(cloud.width) * cloud.point_step ||
    uint64_t(cloud.row_step) * cloud.height > cloud.data.size())
  {
    throw std::invalid_argument("Invalid temporal cloud frame or record layout");
  }
  auto offset = [&](const std::string & name) {
      const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
        [&](const auto & f) {return f.name == name;});
      if (it == cloud.fields.end() || it->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        it->count != 1 || uint64_t(it->offset) + sizeof(float) > cloud.point_step)
      {
        throw std::invalid_argument("Temporal cloud requires FLOAT32 xyz fields");
      }
      return it->offset;
    };
  const auto x_offset = offset("x"), y_offset = offset("y"), z_offset = offset("z");
  auto output = cloud;
  output.data.clear();
  output.height = 1;
  output.width = output.row_step = 0;
  const int64_t stamp = int64_t(cloud.header.stamp.sec) * 1000000000 +
    cloud.header.stamp.nanosec;
  if (last_stamp_ && stamp < *last_stamp_) {reset();}
  if (last_stamp_ && stamp == *last_stamp_) {return output;}
  last_stamp_ = stamp;
  if (!period_start_) {period_start_ = stamp;}
  const auto elapsed = (stamp - *period_start_) / period_ns_;
  if (elapsed > 0) {
    if (elapsed == 1) {
      for (auto it = streaks_.begin(); it != streaks_.end(); ) {
        if (hits_.count(it->first) == 0) {it = streaks_.erase(it);} else {++it;}
      }
      for (auto key : hits_) {
        auto & streak = streaks_[key];
        if (streak < cycles_) {++streak;}
      }
    } else {
      // At least one entire period had no observations; previous hits cannot bridge it.
      streaks_.clear();
    }
    hits_.clear();
    *period_start_ += elapsed * period_ns_;
  }
  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const auto * record = cloud.data.data() + size_t(row) * cloud.row_step +
        size_t(col) * cloud.point_step;
      float x, y, z;
      std::memcpy(&x, record + x_offset, sizeof(float));
      std::memcpy(&y, record + y_offset, sizeof(float));
      std::memcpy(&z, record + z_offset, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      const double dx = x - map_.origin.position.x, dy = y - map_.origin.position.y;
      const double ix = std::floor((cos_yaw_ * dx + sin_yaw_ * dy) / map_.resolution);
      const double iy = std::floor((-sin_yaw_ * dx + cos_yaw_ * dy) / map_.resolution);
      if (ix < 0 || iy < 0 || ix >= map_.width || iy >= map_.height) {continue;}
      const uint64_t key = uint64_t(iy) * map_.width + uint64_t(ix);
      hits_.insert(key);
      const auto it = streaks_.find(key);
      if (it != streaks_.end() && it->second >= cycles_) {
        output.data.insert(output.data.end(), record, record + cloud.point_step);
        ++output.width;
      }
    }
  }
  output.row_step = output.width * output.point_step;
  return output;
}
}  // namespace vlm_nav
