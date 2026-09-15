#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_mapping
{

enum class RobustKernel
{
  HUBER,
  CAUCHY,
};

struct PoseGraphConfig
{
  double prior_rotation_sigma_rad{1e-3};
  double prior_translation_sigma_m{1e-3};
  double odometry_rotation_sigma_rad{0.05};
  double odometry_translation_sigma_m{0.1};
  double loop_rotation_sigma_rad{0.05};
  double loop_translation_sigma_m{0.1};
  RobustKernel robust_kernel{RobustKernel::HUBER};
  double robust_kernel_scale{1.0};
  double relinearization_threshold{0.1};
  int relinearization_skip{1};
};

struct LoopConstraint
{
  std::uint64_t from_id{0U};
  std::uint64_t to_id{0U};
  a2w_fastlio_common::Pose3d relative_pose{};
  bool accepted{false};
};

struct GraphUpdate
{
  bool accepted{false};
  std::string reason{};
  std::size_t node_count{0U};
  std::size_t factor_count{0U};
};

struct OptimizedPose
{
  std::uint64_t id{0U};
  a2w_fastlio_common::Pose3d pose{};
};

struct OptimizedPoseSnapshot
{
  std::uint64_t revision{0U};
  std::vector<OptimizedPose> poses{};
};

gtsam::Pose3 toGtsamPose(const a2w_fastlio_common::Pose3d & pose);
a2w_fastlio_common::Pose3d fromGtsamPose(const gtsam::Pose3 & pose);

class PoseGraphOptimizer
{
public:
  explicit PoseGraphOptimizer(PoseGraphConfig config);
  ~PoseGraphOptimizer();

  PoseGraphOptimizer(const PoseGraphOptimizer &) = delete;
  PoseGraphOptimizer & operator=(const PoseGraphOptimizer &) = delete;
  PoseGraphOptimizer(PoseGraphOptimizer &&) noexcept;
  PoseGraphOptimizer & operator=(PoseGraphOptimizer &&) noexcept;

  GraphUpdate addKeyFrame(
    std::uint64_t id, const a2w_fastlio_common::Pose3d & odom_pose);
  GraphUpdate addLoopConstraint(const LoopConstraint & constraint);
  GraphUpdate update();
  OptimizedPoseSnapshot optimizedPoses() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace a2w_fastlio_mapping
