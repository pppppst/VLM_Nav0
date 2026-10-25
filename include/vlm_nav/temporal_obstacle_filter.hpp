#ifndef VLM_NAV__TEMPORAL_OBSTACLE_FILTER_HPP_
#define VLM_NAV__TEMPORAL_OBSTACLE_FILTER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vlm_nav
{
class TemporalObstacleFilter
{
public:
  TemporalObstacleFilter(double period_seconds, unsigned cycles);
  bool set_map(const nav_msgs::msg::OccupancyGrid & map);
  void reset();
  const std::string & frame() const {return frame_;}
  sensor_msgs::msg::PointCloud2 filter(const sensor_msgs::msg::PointCloud2 & cloud);

private:
  int64_t period_ns_;
  unsigned cycles_;
  nav_msgs::msg::MapMetaData map_;
  std::string frame_;
  double cos_yaw_{1.0}, sin_yaw_{0.0};
  std::optional<int64_t> period_start_, last_stamp_;
  std::unordered_set<uint64_t> hits_;
  std::unordered_map<uint64_t, unsigned> streaks_;
};
}  // namespace vlm_nav
#endif
