#include "a2w_fastlio_common/quatro_registration.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <pcl/common/point_tests.h>
#include <quatro/quatro_module.h>

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

Pose3d poseFromMatrix(const Eigen::Matrix4d & matrix)
{
  Pose3d pose;
  pose.rotation = Eigen::Quaterniond{matrix.block<3, 3>(0, 0)}.normalized();
  pose.translation = matrix.block<3, 1>(0, 3);
  return pose;
}

}  // namespace

QuatroRegistration::QuatroRegistration(QuatroRegistrationConfig config) : config_{config}
{
  if (!(config_.fpfh_normal_radius_m > 0.0) || !(config_.fpfh_radius_m > 0.0) ||
    !(config_.noise_bound_m > 0.0) || !(config_.rotation_gnc_factor > 1.0) ||
    !(config_.rotation_cost_threshold > 0.0) || config_.rotation_max_iterations <= 0 ||
    !(config_.descriptor_distance_threshold > 0.0) || config_.maximum_correspondences <= 0 ||
    config_.minimum_points < 3U)
  {
    throw std::invalid_argument{"invalid Quatro registration configuration"};
  }
}

RegistrationResult QuatroRegistration::align(
  const CloudConstPtr & source, const CloudConstPtr & target,
  const std::optional<Pose3d> & /*initial_guess*/) const
{
  RegistrationResult result;
  if (!validCloud(source, config_.minimum_points) ||
    !validCloud(target, config_.minimum_points))
  {
    result.rejection_reason = "invalid or too-small input cloud";
    return result;
  }

  const auto started = std::chrono::steady_clock::now();
  quatro<PointT> solver{
    config_.fpfh_normal_radius_m, config_.fpfh_radius_m, config_.noise_bound_m,
    config_.rotation_gnc_factor, config_.rotation_cost_threshold,
    config_.rotation_max_iterations, config_.estimate_scale, config_.optimized_matching,
    config_.descriptor_distance_threshold, config_.maximum_correspondences};
  bool valid = false;
  const Eigen::Matrix4d matrix = solver.align(*source, *target, valid);
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  result.transform = poseFromMatrix(matrix);
  result.success = valid && matrix.allFinite() && isFinitePose(result.transform);
  result.converged = result.success;
  result.fitness = result.success ? 0.0 : std::numeric_limits<double>::infinity();
  if (!result.success) {
    result.rejection_reason = "Quatro did not produce a valid transform";
  }
  return result;
}

}  // namespace a2w_fastlio_common
