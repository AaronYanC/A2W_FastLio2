#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/map_odom_manager.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

using a2w_fastlio_common::Pose3d;

Pose3d pose(double x, double y, double yaw)
{
  Pose3d result;
  result.translation = Eigen::Vector3d{x, y, 0.0};
  result.rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return result;
}

void expectPoseNear(const Pose3d & actual, const Pose3d & expected)
{
  EXPECT_NEAR((actual.translation - expected.translation).norm(), 0.0, 1e-10);
  EXPECT_NEAR(
    Eigen::AngleAxisd(expected.rotation.inverse() * actual.rotation).angle(), 0.0, 1e-10);
}

TEST(MapOdomManager, IdentityPosesProduceIdentityCorrection)
{
  MapOdomManager manager;
  const auto correction = manager.update(0U, 100LL, Pose3d{}, Pose3d{});
  expectPoseNear(correction.map_camera_init, Pose3d{});
  EXPECT_EQ(correction.keyframe_id, 0U);
  EXPECT_EQ(correction.stamp_ns, 100LL);
  EXPECT_EQ(correction.ageNs(160LL), 60LL);
}

TEST(MapOdomManager, ComputesMapBodyTimesInverseCameraInitBody)
{
  MapOdomManager manager;
  const auto map_body = pose(4.0, 2.0, 0.7);
  const auto camera_init_body = pose(1.0, -0.5, 0.2);

  const auto correction = manager.update(7U, 200LL, map_body, camera_init_body);

  const auto recomposed = composePose(correction.map_camera_init, camera_init_body);
  expectPoseNear(recomposed, map_body);
  const auto latest = manager.latest();
  ASSERT_TRUE(latest.has_value());
  expectPoseNear(latest->map_camera_init, correction.map_camera_init);
}

TEST(MapOdomManager, RejectsNonMonotonicTimestampWithoutReplacingSnapshot)
{
  MapOdomManager manager;
  const auto first = manager.update(0U, 100LL, Pose3d{}, Pose3d{});
  EXPECT_THROW(manager.update(1U, 100LL, pose(1.0, 0.0, 0.0), Pose3d{}), std::invalid_argument);
  EXPECT_THROW(manager.update(1U, 99LL, pose(1.0, 0.0, 0.0), Pose3d{}), std::invalid_argument);
  ASSERT_TRUE(manager.latest().has_value());
  EXPECT_EQ(manager.latest()->keyframe_id, first.keyframe_id);
}

TEST(MapOdomManager, RejectsNonIncreasingKeyframeIdAndNegativeAge)
{
  MapOdomManager manager;
  const auto correction = manager.update(2U, 100LL, Pose3d{}, Pose3d{});
  EXPECT_THROW(manager.update(2U, 101LL, Pose3d{}, Pose3d{}), std::invalid_argument);
  EXPECT_THROW(correction.ageNs(99LL), std::invalid_argument);
}

TEST(MapOdomManager, RejectsNonFinitePoseWithoutCreatingSnapshot)
{
  MapOdomManager manager;
  auto invalid = Pose3d{};
  invalid.translation.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(manager.update(0U, 100LL, invalid, Pose3d{}), std::invalid_argument);
  EXPECT_FALSE(manager.latest().has_value());
}

}  // namespace
}  // namespace a2w_fastlio_mapping
