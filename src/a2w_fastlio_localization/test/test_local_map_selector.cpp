#include <gtest/gtest.h>

#include "a2w_fastlio_localization/local_map_selector.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_map::MapSnapshot snapshot()
{
  a2w_fastlio_map::MapSnapshot result;
  for (std::uint64_t id = 0U; id < 5U; ++id) {
    a2w_fastlio_common::KeyFrame frame;
    frame.id = id;
    frame.stamp_ns = id + 1U;
    frame.optimized_pose.translation.x() = static_cast<double>(id) * 2.0;
    frame.body_cloud->push_back(a2w_fastlio_common::PointT{});
    result.keyframes.push_back(frame);
  }
  return result;
}

TEST(LocalMapSelector, SelectsRadiusLimitedKNearestWithStableTieOrder)
{
  LocalMapSelector selector{snapshot(), LocalMapSelectorConfig{2.1, 2U, 1U}};
  a2w_fastlio_common::Pose3d pose;
  pose.translation.x() = 3.0;
  const auto selected = selector.select(pose);
  ASSERT_TRUE(selected.success);
  ASSERT_EQ(selected.keyframe_ids.size(), 2U);
  EXPECT_EQ(selected.keyframe_ids[0], 1U);
  EXPECT_EQ(selected.keyframe_ids[1], 2U);
  EXPECT_EQ(selected.nearest_keyframe_id, 1U);
  EXPECT_DOUBLE_EQ(selected.initial_map_body.translation.x(), 3.0);
}

TEST(LocalMapSelector, IncludesRadiusBoundaryAndReportsNoNeighbor)
{
  LocalMapSelector selector{snapshot(), LocalMapSelectorConfig{2.0, 3U, 1U}};
  a2w_fastlio_common::Pose3d pose;
  const auto boundary = selector.select(pose);
  ASSERT_TRUE(boundary.success);
  EXPECT_EQ(boundary.keyframe_ids, (std::vector<std::uint64_t>{0U, 1U}));
  pose.translation.x() = 100.0;
  const auto missing = selector.select(pose);
  EXPECT_FALSE(missing.success);
  EXPECT_EQ(missing.reason, "no_nearby_keyframes");
}

TEST(LocalMapSelector, RejectsInvalidConfigurationAndNonFinitePose)
{
  EXPECT_THROW(LocalMapSelector(snapshot(), {0.0, 1U, 1U}), std::invalid_argument);
  a2w_fastlio_common::Pose3d pose;
  pose.translation.x() = NAN;
  EXPECT_FALSE(LocalMapSelector(snapshot(), {2.0, 1U, 1U}).select(pose).success);
}

}  // namespace
}  // namespace a2w_fastlio_localization
