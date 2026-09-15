#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

#include "a2w_fastlio_common/point_types.hpp"
#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_common
{

struct RegistrationResult
{
  bool success{false};
  bool converged{false};
  Pose3d transform{};
  double fitness{std::numeric_limits<double>::infinity()};
  double overlap{0.0};
  std::size_t correspondence_count{0U};
  double elapsed_ms{0.0};
  std::string rejection_reason{};
};

inline bool isFinitePose(const Pose3d & pose) noexcept
{
  const auto & q = pose.rotation;
  return pose.translation.allFinite() && q.coeffs().allFinite() &&
         std::isfinite(q.norm()) && q.norm() > 1e-12;
}

inline bool isUsableRegistration(const RegistrationResult & result) noexcept
{
  return result.success && result.converged && isFinitePose(result.transform) &&
         std::isfinite(result.fitness) && result.fitness >= 0.0 &&
         std::isfinite(result.overlap) && result.overlap >= 0.0 && result.overlap <= 1.0 &&
         std::isfinite(result.elapsed_ms) && result.elapsed_ms >= 0.0;
}

class CoarseRegistration
{
public:
  virtual ~CoarseRegistration() = default;
  virtual RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & initial_guess) const = 0;
};

class FineRegistration
{
public:
  virtual ~FineRegistration() = default;
  virtual RegistrationResult align(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & initial_guess) const = 0;
};

}  // namespace a2w_fastlio_common
