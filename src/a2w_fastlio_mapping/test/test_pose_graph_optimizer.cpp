#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

using a2w_fastlio_common::Pose3d;

Pose3d pose(double x, double y, double z, double yaw)
{
  Pose3d result;
  result.translation = Eigen::Vector3d{x, y, z};
  result.rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return result;
}

void expectPoseNear(const Pose3d & actual, const Pose3d & expected, double tolerance = 1e-8)
{
  EXPECT_NEAR((actual.translation - expected.translation).norm(), 0.0, tolerance);
  EXPECT_NEAR(
    Eigen::AngleAxisd(expected.rotation.inverse() * actual.rotation).angle(), 0.0, tolerance);
}

TEST(PoseGraphOptimizer, PoseConversionRoundTripsWithoutChangingConvention)
{
  const auto original = pose(1.25, -2.5, 0.75, 0.42);
  expectPoseNear(fromGtsamPose(toGtsamPose(original)), original);
}

TEST(PoseGraphOptimizer, FirstKeyFrameCreatesPriorAndSnapshot)
{
  PoseGraphOptimizer optimizer{PoseGraphConfig{}};
  const auto initial = pose(4.0, -1.0, 0.2, 0.3);

  EXPECT_TRUE(optimizer.addKeyFrame(0U, initial).accepted);
  const auto update = optimizer.update();
  const auto snapshot = optimizer.optimizedPoses();

  EXPECT_TRUE(update.accepted) << update.reason;
  EXPECT_EQ(snapshot.revision, 1U);
  ASSERT_EQ(snapshot.poses.size(), 1U);
  EXPECT_EQ(snapshot.poses.front().id, 0U);
  expectPoseNear(snapshot.poses.front().pose, initial, 1e-6);
}

TEST(PoseGraphOptimizer, AdjacentOdometryFactorsPreserveRelativeMotion)
{
  PoseGraphOptimizer optimizer{PoseGraphConfig{}};
  const auto p0 = pose(0.0, 0.0, 0.0, 0.0);
  const auto p1 = pose(1.0, 0.2, 0.0, 0.1);
  const auto p2 = pose(2.1, 0.4, 0.0, 0.2);
  ASSERT_TRUE(optimizer.addKeyFrame(0U, p0).accepted);
  ASSERT_TRUE(optimizer.addKeyFrame(1U, p1).accepted);
  ASSERT_TRUE(optimizer.addKeyFrame(2U, p2).accepted);

  ASSERT_TRUE(optimizer.update().accepted);
  const auto snapshot = optimizer.optimizedPoses();

  ASSERT_EQ(snapshot.poses.size(), 3U);
  expectPoseNear(snapshot.poses[0].pose, p0, 1e-5);
  expectPoseNear(snapshot.poses[1].pose, p1, 1e-5);
  expectPoseNear(snapshot.poses[2].pose, p2, 1e-5);
}

TEST(PoseGraphOptimizer, RejectsNonContiguousIdsWithoutMutation)
{
  PoseGraphOptimizer optimizer{PoseGraphConfig{}};
  ASSERT_TRUE(optimizer.addKeyFrame(0U, Pose3d{}).accepted);
  const auto rejected = optimizer.addKeyFrame(2U, pose(2.0, 0.0, 0.0, 0.0));

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "keyframe_id_not_contiguous");
  ASSERT_TRUE(optimizer.update().accepted);
  EXPECT_EQ(optimizer.optimizedPoses().poses.size(), 1U);
}

TEST(PoseGraphOptimizer, RejectsNonFinitePoseWithoutMutation)
{
  PoseGraphOptimizer optimizer{PoseGraphConfig{}};
  auto invalid = Pose3d{};
  invalid.translation.x() = std::numeric_limits<double>::quiet_NaN();

  const auto rejected = optimizer.addKeyFrame(0U, invalid);

  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.reason, "keyframe_pose_not_finite");
  EXPECT_FALSE(optimizer.update().accepted);
  EXPECT_TRUE(optimizer.optimizedPoses().poses.empty());
}

TEST(PoseGraphOptimizer, EmptyUpdateDoesNotAdvanceRevision)
{
  PoseGraphOptimizer optimizer{PoseGraphConfig{}};
  const auto update = optimizer.update();
  EXPECT_FALSE(update.accepted);
  EXPECT_EQ(update.reason, "no_pending_factors");
  EXPECT_EQ(optimizer.optimizedPoses().revision, 0U);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
