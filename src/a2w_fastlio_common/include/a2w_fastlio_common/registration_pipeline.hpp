#pragma once

#include <memory>
#include <optional>

#include "a2w_fastlio_common/match_validator.hpp"

namespace a2w_fastlio_common
{

struct RegistrationPipelineConfig
{
  double evidence_distance_m{0.5};
};

struct PipelineResult
{
  bool success{false};
  RegistrationResult coarse{};
  RegistrationResult fine{};
  ValidationResult validation{};
  Pose3d final_transform{};
};

class RegistrationPipeline
{
public:
  RegistrationPipeline(
    std::shared_ptr<const CoarseRegistration> coarse,
    std::shared_ptr<const FineRegistration> fine,
    MatchValidator validator,
    RegistrationPipelineConfig config);

  PipelineResult run(
    const CloudConstPtr & source, const CloudConstPtr & target,
    const std::optional<Pose3d> & yaw_hint,
    const ValidationContext & context = {}) const;

private:
  RegistrationResult withGeometricEvidence(
    RegistrationResult result, const CloudConstPtr & source,
    const CloudConstPtr & target) const;

  std::shared_ptr<const CoarseRegistration> coarse_;
  std::shared_ptr<const FineRegistration> fine_;
  MatchValidator validator_;
  RegistrationPipelineConfig config_;
};

}  // namespace a2w_fastlio_common
