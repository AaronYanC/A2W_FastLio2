#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

using a2w_fastlio_common::Pose3d;

Pose3d pose(double x, double y, double yaw = 0.0)
{
  Pose3d result;
  result.translation = Eigen::Vector3d{x, y, 0.0};
  result.rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return result;
}

double trajectoryError(
  const OptimizedPoseSnapshot & snapshot, const std::vector<Pose3d> & truth)
{
  double error = 0.0;
  for (std::size_t index = 0; index < truth.size(); ++index) {
    error += (snapshot.poses.at(index).pose.translation - truth[index].translation).norm();
  }
  return error;
}

PoseGraphConfig preciseLoopConfig()
{
  PoseGraphConfig config;
  config.odometry_translation_sigma_m = 0.2;
  config.odometry_rotation_sigma_rad = 0.1;
  config.loop_translation_sigma_m = 0.01;
  config.loop_rotation_sigma_rad = 0.01;
  config.robust_kernel = RobustKernel::HUBER;
  config.robust_kernel_scale = 5.0;
  return config;
}

TEST(PoseGraphSyntheticLoop, CorrectLoopReducesEndpointAndTrajectoryError)
{
  const std::vector<Pose3d> truth{
    pose(0.0, 0.0), pose(1.0, 0.0), pose(1.0, 1.0), pose(0.0, 1.0), pose(0.0, 0.0)};
  const std::vector<Pose3d> drifted{
    pose(0.0, 0.0), pose(1.05, 0.0), pose(1.1, 1.05), pose(0.1, 1.1), pose(0.3, 0.2)};
  PoseGraphOptimizer optimizer{preciseLoopConfig()};
  for (std::uint64_t id = 0U; id < drifted.size(); ++id) {
    ASSERT_TRUE(optimizer.addKeyFrame(id, drifted[id]).accepted);
  }
  ASSERT_TRUE(optimizer.update().accepted);
  const auto before = optimizer.optimizedPoses();

  LoopConstraint closure;
  closure.from_id = 4U;
  closure.to_id = 0U;
  closure.relative_pose = Pose3d{};
  closure.accepted = true;
  ASSERT_TRUE(optimizer.addLoopConstraint(closure).accepted);
  ASSERT_TRUE(optimizer.update().accepted);
  const auto after = optimizer.optimizedPoses();

  EXPECT_LT(
    after.poses.back().pose.translation.norm(),
    before.poses.back().pose.translation.norm());
  EXPECT_LT(trajectoryError(after, truth), trajectoryError(before, truth));
  EXPECT_EQ(after.revision, before.revision + 1U);
}

TEST(PoseGraphSyntheticLoop, CauchyKernelLimitsBadLoopDisplacement)
{
  PoseGraphConfig config = preciseLoopConfig();
  config.robust_kernel = RobustKernel::CAUCHY;
  config.robust_kernel_scale = 0.2;
  PoseGraphOptimizer optimizer{config};
  for (std::uint64_t id = 0U; id < 5U; ++id) {
    ASSERT_TRUE(optimizer.addKeyFrame(id, pose(static_cast<double>(id), 0.0)).accepted);
  }
  ASSERT_TRUE(optimizer.update().accepted);
  const auto before = optimizer.optimizedPoses();

  LoopConstraint bad_loop;
  bad_loop.from_id = 4U;
  bad_loop.to_id = 0U;
  bad_loop.relative_pose = pose(25.0, 0.0);
  bad_loop.accepted = true;
  ASSERT_TRUE(optimizer.addLoopConstraint(bad_loop).accepted);
  ASSERT_TRUE(optimizer.update().accepted);
  const auto after = optimizer.optimizedPoses();

  const double displacement =
    (after.poses.back().pose.translation - before.poses.back().pose.translation).norm();
  EXPECT_LT(displacement, 0.25);
}

TEST(PoseGraphSyntheticLoop, RejectsUnvalidatedInvalidAndOutOfRangeLoops)
{
  PoseGraphOptimizer optimizer{PoseGraphConfig{}};
  ASSERT_TRUE(optimizer.addKeyFrame(0U, Pose3d{}).accepted);
  ASSERT_TRUE(optimizer.addKeyFrame(1U, pose(1.0, 0.0)).accepted);
  ASSERT_TRUE(optimizer.update().accepted);
  const auto factor_count = optimizer.update().factor_count;

  LoopConstraint loop;
  loop.from_id = 1U;
  loop.to_id = 0U;
  EXPECT_EQ(optimizer.addLoopConstraint(loop).reason, "loop_not_validated");
  loop.accepted = true;
  loop.from_id = 2U;
  EXPECT_EQ(optimizer.addLoopConstraint(loop).reason, "loop_keyframe_out_of_range");
  loop.from_id = 1U;
  loop.to_id = 1U;
  EXPECT_EQ(optimizer.addLoopConstraint(loop).reason, "loop_keyframes_not_distinct");
  loop.to_id = 0U;
  loop.relative_pose.translation.x() = std::numeric_limits<double>::infinity();
  EXPECT_EQ(optimizer.addLoopConstraint(loop).reason, "loop_pose_not_finite");
  EXPECT_EQ(optimizer.update().factor_count, factor_count);
}

TEST(PoseGraphSyntheticLoop, RejectsInvalidNoiseAndIsamConfiguration)
{
  auto config = PoseGraphConfig{};
  config.loop_translation_sigma_m = 0.0;
  EXPECT_THROW((PoseGraphOptimizer{config}), std::invalid_argument);
  config = PoseGraphConfig{};
  config.relinearization_skip = 0;
  EXPECT_THROW((PoseGraphOptimizer{config}), std::invalid_argument);
  config = PoseGraphConfig{};
  config.robust_kernel_scale = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((PoseGraphOptimizer{config}), std::invalid_argument);
  config = PoseGraphConfig{};
  config.robust_kernel = static_cast<RobustKernel>(99);
  EXPECT_THROW((PoseGraphOptimizer{config}), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
