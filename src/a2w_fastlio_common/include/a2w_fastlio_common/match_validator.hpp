#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_common
{

struct MatchValidationConfig
{
  double maximum_fitness{0.25};
  double minimum_overlap{0.35};
  std::size_t minimum_correspondences{30U};
  double maximum_translation_jump_m{5.0};
  double maximum_rotation_jump_rad{1.0};
  double minimum_candidate_distance_separation{0.05};
};

struct ValidationContext
{
  std::optional<Pose3d> predicted_transform{};
  std::optional<double> best_descriptor_distance{};
  std::optional<double> second_descriptor_distance{};
};

struct ValidationResult
{
  bool accepted{false};
  std::string reason{};
};

class MatchValidator
{
public:
  explicit MatchValidator(MatchValidationConfig config);

  ValidationResult validate(
    const RegistrationResult & result, const ValidationContext & context) const;

private:
  MatchValidationConfig config_;
};

using LoopValidator = MatchValidator;

}  // namespace a2w_fastlio_common
