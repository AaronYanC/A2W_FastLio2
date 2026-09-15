#include "a2w_fastlio_common/nano_gicp_registration.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <nano_gicp/nano_gicp.hpp>
#include <pcl/common/point_tests.h>

namespace a2w_fastlio_common
{
namespace
{

bool validCloud(const CloudConstPtr & cloud, const std::size_t minimum_points)
{
  if (!cloud || cloud->size() < minimum_points) {
    return false;
  }
  for (const auto & point : cloud->points) {
    if (!pcl::isFinite(point)) {
      return false;
    }
  }
  return true;
}

Eigen::Matrix4f matrixFromPose(const Pose3d & pose)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = pose.rotation.toRotationMatrix().cast<float>();
  matrix.block<3, 1>(0, 3) = pose.translation.cast<float>();
  return matrix;
}

Pose3d poseFromMatrix(const Eigen::Matrix4f & matrix)
{
  Pose3d pose;
  pose.rotation = Eigen::Quaterniond{matrix.block<3, 3>(0, 0).cast<double>()}.normalized();
  pose.translation = matrix.block<3, 1>(0, 3).cast<double>();
  return pose;
}

}  // namespace

NanoGicpRegistration::NanoGicpRegistration(NanoGicpRegistrationConfig config) : config_{config}
{
  if (!(config_.maximum_correspondence_distance_m > 0.0) || config_.thread_count < 0 ||
    config_.correspondence_randomness < 3 || config_.maximum_iterations <= 0 ||
    !(config_.transformation_epsilon > 0.0) || !(config_.rotation_epsilon > 0.0) ||
    config_.regularization_method < 0 || config_.regularization_method > 4 ||
    !(config_.fitness_score_max_range_m > 0.0) || config_.minimum_points < 3U)
  {
    throw std::invalid_argument{"invalid Nano-GICP registration configuration"};
  }
}

RegistrationResult NanoGicpRegistration::align(
  const CloudConstPtr & source, const CloudConstPtr & target,
  const std::optional<Pose3d> & initial_guess) const
{
  RegistrationResult result;
  if (!validCloud(source, config_.minimum_points) ||
    !validCloud(target, config_.minimum_points) ||
    (initial_guess && !isFinitePose(*initial_guess)))
  {
    result.rejection_reason = "invalid or too-small input cloud/guess";
    return result;
  }

  const auto started = std::chrono::steady_clock::now();
  nano_gicp::NanoGICP<PointT, PointT> solver;
  solver.setNumThreads(config_.thread_count);
  solver.setCorrespondenceRandomness(config_.correspondence_randomness);
  solver.setMaximumIterations(config_.maximum_iterations);
  solver.setMaxCorrespondenceDistance(config_.maximum_correspondence_distance_m);
  solver.setTransformationEpsilon(config_.transformation_epsilon);
  solver.setRotationEpsilon(config_.rotation_epsilon);
  solver.setRegularizationMethod(
    static_cast<nano_gicp::RegularizationMethod>(config_.regularization_method));
  solver.setInputSource(source);
  solver.setInputTarget(target);
  Cloud aligned;
  solver.align(aligned, initial_guess ? matrixFromPose(*initial_guess) : Eigen::Matrix4f::Identity());

  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.transform = poseFromMatrix(solver.getFinalTransformation());
  result.converged = solver.hasConverged();
  result.fitness = solver.getFitnessScore(config_.fitness_score_max_range_m);
  result.success = result.converged && isFinitePose(result.transform) &&
    std::isfinite(result.fitness) && result.fitness >= 0.0;
  if (!result.success) {
    result.rejection_reason = "Nano-GICP did not converge to a finite transform";
  }
  return result;
}

}  // namespace a2w_fastlio_common
