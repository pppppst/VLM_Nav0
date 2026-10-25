#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
#include "vlm_nav/temporal_obstacle_filter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
nav_msgs::msg::OccupancyGrid map()
{
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = "map";
  m.info.resolution = .05;
  m.info.width = m.info.height = 200;
  m.info.origin.position.x = m.info.origin.position.y = -5.;
  m.info.origin.orientation.w = 1.;
  return m;
}
sensor_msgs::msg::PointCloud2 cloud(int64_t ns, std::vector<float> xs = {1.01F})
{
  sensor_msgs::msg::PointCloud2 c;
  c.header.frame_id = "map";
  c.header.stamp.sec = ns / 1000000000;
  c.header.stamp.nanosec = ns % 1000000000;
  sensor_msgs::PointCloud2Modifier m(c);
  m.setPointCloud2Fields(4, "x", 1, 7, "y", 1, 7, "z", 1, 7, "intensity", 1, 7);
  m.resize(xs.size());
  sensor_msgs::PointCloud2Iterator<float> x(c,"x"), y(c,"y"), z(c,"z"), i(c,"intensity");
  for (float value : xs) { *x=value; *y=.01; *z=.1; *i=17.; ++x; ++y; ++z; ++i; }
  return c;
}
constexpr int64_t P = 800000000;
}

TEST(TemporalObstacleFilter, RequiresThreeCompletePeriodsAndPreservesCurrentRecords)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); f.set_map(map());
  for (int64_t t=0;t<3*P;t+=100000000) EXPECT_EQ(f.filter(cloud(t)).width,0u);
  auto input=cloud(3*P), out=f.filter(input);
  EXPECT_EQ(out.width,1u); EXPECT_EQ(out.header,input.header);
  EXPECT_EQ(out.fields,input.fields); EXPECT_EQ(out.data,input.data);
}
TEST(TemporalObstacleFilter, DuplicateAndDenseFramesDoNotCountAsCycles)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); f.set_map(map());
  for (int i=0;i<100;++i) EXPECT_EQ(f.filter(cloud(0,{1.01F,1.02F})).width,0u);
  EXPECT_EQ(f.filter(cloud(P)).width,0u);
  EXPECT_EQ(f.filter(cloud(2*P)).width,0u);
  EXPECT_EQ(f.filter(cloud(3*P)).width,1u);
}
TEST(TemporalObstacleFilter, MissingPeriodRestartsConsecutiveCount)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); f.set_map(map());
  f.filter(cloud(0)); f.filter(cloud(P)); f.filter(cloud(2*P,{}));
  for (int k=3;k<6;++k) EXPECT_EQ(f.filter(cloud(k*P)).width,0u);
  EXPECT_EQ(f.filter(cloud(6*P)).width,1u);
}
TEST(TemporalObstacleFilter, StreamGapAndBackwardStampRestartConfirmation)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); f.set_map(map());
  for (int k=0;k<=3;++k) f.filter(cloud(k*P));
  EXPECT_EQ(f.filter(cloud(10*P)).width,0u);
  for (int k=11;k<13;++k) EXPECT_EQ(f.filter(cloud(k*P)).width,0u);
  EXPECT_EQ(f.filter(cloud(13*P)).width,1u);
  EXPECT_EQ(f.filter(cloud(0)).width,0u);
}
TEST(TemporalObstacleFilter, CountsDistinctMapCellsAndNeverReplaysVanishedPoints)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); f.set_map(map());
  f.filter(cloud(0,{1.01F})); f.filter(cloud(P,{1.02F})); f.filter(cloud(2*P,{1.03F}));
  auto out=f.filter(cloud(3*P,{1.04F,1.07F}));
  ASSERT_EQ(out.width,1u);
  EXPECT_FLOAT_EQ(*sensor_msgs::PointCloud2ConstIterator<float>(out,"x"),1.04F);
  EXPECT_EQ(f.filter(cloud(3*P+1,{})).width,0u);
}
TEST(TemporalObstacleFilter, MapGeometryChangeResetsButRepublishingSameMapDoesNot)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); auto m=map(); f.set_map(m);
  for (int k=0;k<=3;++k) { f.set_map(m); auto c=f.filter(cloud(k*P)); EXPECT_EQ(c.width,k==3?1u:0u); }
  m.info.origin.position.x+=.02; f.set_map(m);
  EXPECT_EQ(f.filter(cloud(4*P)).width,0u);
}
TEST(TemporalObstacleFilter, UsesRotatedOriginAndIgnoresOutsideAndNonfinitePoints)
{
  vlm_nav::TemporalObstacleFilter f(.8,3); auto m=map();
  m.info.origin.position.x=2.;m.info.origin.position.y=-1.;
  m.info.origin.orientation.z=std::sin(M_PI/4);m.info.origin.orientation.w=std::cos(M_PI/4);
  f.set_map(m);
  for (int k=0;k<3;++k) f.filter(cloud(k*P,{1.01F}));
  EXPECT_EQ(f.filter(cloud(3*P,{1.01F,3.F,std::numeric_limits<float>::quiet_NaN()})).width,1u);
}
TEST(TemporalObstacleFilter, RejectsInvalidConfigurationMapAndCloudFrame)
{
  EXPECT_THROW(vlm_nav::TemporalObstacleFilter(0,3),std::invalid_argument);
  EXPECT_THROW(vlm_nav::TemporalObstacleFilter(.8,0),std::invalid_argument);
  vlm_nav::TemporalObstacleFilter f(.8,3);
  EXPECT_THROW(f.filter(cloud(0)),std::logic_error);
  auto m=map(); m.info.resolution=0;
  EXPECT_THROW(f.set_map(m),std::invalid_argument);
  f.set_map(map());auto c=cloud(0);c.header.frame_id="base_link";
  EXPECT_THROW(f.filter(c),std::invalid_argument);
}
