#include "a2w_fastlio_mapping/pose_graph_optimizer.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

gtsam::Key key(const std::uint64_t id)
{
  return gtsam::Symbol{'x', id};
}

gtsam::SharedNoiseModel diagonalNoise(double rotation_sigma, double translation_sigma)
{
  gtsam::Vector6 sigmas;
  sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
    translation_sigma, translation_sigma, translation_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

bool validConfig(const PoseGraphConfig & config)
{
  const bool valid_kernel = config.robust_kernel == RobustKernel::HUBER ||
    config.robust_kernel == RobustKernel::CAUCHY;
  return valid_kernel &&
         config.prior_rotation_sigma_rad > 0.0 && config.prior_translation_sigma_m > 0.0 &&
         config.odometry_rotation_sigma_rad > 0.0 &&
         config.odometry_translation_sigma_m > 0.0 && config.loop_rotation_sigma_rad > 0.0 &&
         config.loop_translation_sigma_m > 0.0 && config.robust_kernel_scale > 0.0 &&
         config.relinearization_threshold > 0.0 && config.relinearization_skip > 0;
}

gtsam::SharedNoiseModel robustLoopNoise(const PoseGraphConfig & config)
{
  gtsam::noiseModel::mEstimator::Base::shared_ptr loss;
  switch (config.robust_kernel) {
    case RobustKernel::HUBER:
      loss = gtsam::noiseModel::mEstimator::Huber::Create(config.robust_kernel_scale);
      break;
    case RobustKernel::CAUCHY:
      loss = gtsam::noiseModel::mEstimator::Cauchy::Create(config.robust_kernel_scale);
      break;
    default:
      throw std::invalid_argument{"unsupported robust kernel"};
  }
  return gtsam::noiseModel::Robust::Create(
    loss, diagonalNoise(config.loop_rotation_sigma_rad, config.loop_translation_sigma_m));
}

}  // namespace

gtsam::Pose3 toGtsamPose(const a2w_fastlio_common::Pose3d & pose)
{
  if (!a2w_fastlio_common::isFinitePose(pose)) {
    throw std::invalid_argument{"cannot convert a non-finite pose"};
  }
  const auto quaternion = pose.rotation.normalized();
  return gtsam::Pose3{
    gtsam::Rot3::Quaternion(
      quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()),
    gtsam::Point3{pose.translation.x(), pose.translation.y(), pose.translation.z()}};
}

a2w_fastlio_common::Pose3d fromGtsamPose(const gtsam::Pose3 & pose)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation().toQuaternion().normalized();
  result.translation = pose.translation();
  if (!a2w_fastlio_common::isFinitePose(result)) {
    throw std::runtime_error{"GTSAM produced a non-finite pose"};
  }
  return result;
}

struct PoseGraphOptimizer::Impl
{
  explicit Impl(const PoseGraphConfig & input_config)
  : config{input_config}, isam{isamParameters(input_config)} {}

  static gtsam::ISAM2Params isamParameters(const PoseGraphConfig & config)
  {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = config.relinearization_threshold;
    parameters.relinearizeSkip = config.relinearization_skip;
    return parameters;
  }

  GraphUpdate state(bool accepted, std::string reason) const
  {
    return {accepted, std::move(reason), odometry_poses.size(), factor_count};
  }

  PoseGraphConfig config;
  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph pending_factors;
  gtsam::Values pending_values;
  std::vector<a2w_fastlio_common::Pose3d> odometry_poses;
  std::size_t factor_count{0U};
  OptimizedPoseSnapshot snapshot;
};

PoseGraphOptimizer::PoseGraphOptimizer(PoseGraphConfig config)
{
  if (!validConfig(config)) {
    throw std::invalid_argument{"invalid pose-graph configuration"};
  }
  impl_ = std::make_unique<Impl>(config);
}

PoseGraphOptimizer::~PoseGraphOptimizer() = default;
PoseGraphOptimizer::PoseGraphOptimizer(PoseGraphOptimizer &&) noexcept = default;
PoseGraphOptimizer & PoseGraphOptimizer::operator=(PoseGraphOptimizer &&) noexcept = default;

GraphUpdate PoseGraphOptimizer::addKeyFrame(
  const std::uint64_t id, const a2w_fastlio_common::Pose3d & odom_pose)
{
  if (!a2w_fastlio_common::isFinitePose(odom_pose)) {
    return impl_->state(false, "keyframe_pose_not_finite");
  }
  if (id != impl_->odometry_poses.size()) {
    return impl_->state(false, "keyframe_id_not_contiguous");
  }

  const auto current = toGtsamPose(odom_pose);
  if (id == 0U) {
    impl_->pending_factors.add(gtsam::PriorFactor<gtsam::Pose3>{
      key(id), current,
      diagonalNoise(
        impl_->config.prior_rotation_sigma_rad, impl_->config.prior_translation_sigma_m)});
  } else {
    const auto previous = toGtsamPose(impl_->odometry_poses.back());
    impl_->pending_factors.add(gtsam::BetweenFactor<gtsam::Pose3>{
      key(id - 1U), key(id), previous.between(current),
      diagonalNoise(
        impl_->config.odometry_rotation_sigma_rad,
        impl_->config.odometry_translation_sigma_m)});
  }
  impl_->pending_values.insert(key(id), current);
  impl_->odometry_poses.push_back(odom_pose);
  ++impl_->factor_count;
  return impl_->state(true, "queued");
}

GraphUpdate PoseGraphOptimizer::addLoopConstraint(const LoopConstraint & constraint)
{
  if (!constraint.accepted) {
    return impl_->state(false, "loop_not_validated");
  }
  if (constraint.from_id >= impl_->odometry_poses.size() ||
    constraint.to_id >= impl_->odometry_poses.size())
  {
    return impl_->state(false, "loop_keyframe_out_of_range");
  }
  if (constraint.from_id == constraint.to_id) {
    return impl_->state(false, "loop_keyframes_not_distinct");
  }
  if (!a2w_fastlio_common::isFinitePose(constraint.relative_pose)) {
    return impl_->state(false, "loop_pose_not_finite");
  }

  impl_->pending_factors.add(gtsam::BetweenFactor<gtsam::Pose3>{
    key(constraint.from_id), key(constraint.to_id), toGtsamPose(constraint.relative_pose),
    robustLoopNoise(impl_->config)});
  ++impl_->factor_count;
  return impl_->state(true, "queued");
}

GraphUpdate PoseGraphOptimizer::update()
{
  if (impl_->pending_factors.empty()) {
    return impl_->state(false, "no_pending_factors");
  }

  try {
    impl_->isam.update(impl_->pending_factors, impl_->pending_values);
    const auto estimate = impl_->isam.calculateEstimate();
    OptimizedPoseSnapshot next;
    next.revision = impl_->snapshot.revision + 1U;
    next.poses.reserve(impl_->odometry_poses.size());
    for (std::uint64_t id = 0U; id < impl_->odometry_poses.size(); ++id) {
      next.poses.push_back({id, fromGtsamPose(estimate.at<gtsam::Pose3>(key(id)))});
    }
    impl_->pending_factors.resize(0U);
    impl_->pending_values.clear();
    impl_->snapshot = std::move(next);
    return impl_->state(true, "updated");
  } catch (const std::exception & error) {
    return impl_->state(false, std::string{"gtsam_update_failed:"} + error.what());
  }
}

OptimizedPoseSnapshot PoseGraphOptimizer::optimizedPoses() const
{
  return impl_->snapshot;
}

}  // namespace a2w_fastlio_mapping
