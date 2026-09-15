#include "a2w_fastlio_common/match_validator.hpp"

#include <cmath>
#include <stdexcept>

namespace a2w_fastlio_common
{

MatchValidator::MatchValidator(MatchValidationConfig config) : config_{config}
{
  if (!(config_.maximum_fitness >= 0.0) ||
    !(config_.minimum_overlap >= 0.0 && config_.minimum_overlap <= 1.0) ||
    config_.minimum_correspondences == 0U || !(config_.maximum_translation_jump_m > 0.0) ||
    !(config_.maximum_rotation_jump_rad > 0.0 && config_.maximum_rotation_jump_rad <= M_PI) ||
    !(config_.minimum_candidate_distance_separation >= 0.0))
  {
    throw std::invalid_argument{"invalid match-validation configuration"};
  }
}

ValidationResult MatchValidator::validate(
  const RegistrationResult & result, const ValidationContext & context) const
{
  if (!isUsableRegistration(result)) {
    return {false, "registration_not_usable"};
  }
  if (result.fitness > config_.maximum_fitness) {
    return {false, "fitness_above_limit"};
  }
  if (result.overlap < config_.minimum_overlap) {
    return {false, "overlap_below_limit"};
  }
  if (result.correspondence_count < config_.minimum_correspondences) {
    return {false, "correspondences_below_limit"};
  }

  if (context.predicted_transform) {
    const auto & predicted = *context.predicted_transform;
    if (!isFinitePose(predicted)) {
      return {false, "prediction_not_finite"};
    }
    if ((result.transform.translation - predicted.translation).norm() >
      config_.maximum_translation_jump_m)
    {
      return {false, "translation_jump_above_limit"};
    }
    const double rotation_jump = Eigen::AngleAxisd(
      predicted.rotation.inverse() * result.transform.rotation).angle();
    if (!std::isfinite(rotation_jump) || rotation_jump > config_.maximum_rotation_jump_rad) {
      return {false, "rotation_jump_above_limit"};
    }
  }

  if (context.best_descriptor_distance && context.second_descriptor_distance) {
    const double best = *context.best_descriptor_distance;
    const double second = *context.second_descriptor_distance;
    if (!std::isfinite(best) || !std::isfinite(second) || best < 0.0 || second < best) {
      return {false, "candidate_distances_invalid"};
    }
    if (second - best < config_.minimum_candidate_distance_separation) {
      return {false, "candidate_separation_below_limit"};
    }
  }
  return {true, "accepted"};
}

}  // namespace a2w_fastlio_common
