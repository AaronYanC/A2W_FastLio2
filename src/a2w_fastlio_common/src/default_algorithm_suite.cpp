#include "a2w_fastlio_common/default_algorithm_suite.hpp"

#include <memory>

#include "a2w_fastlio_common/match_validator.hpp"
#include "a2w_fastlio_common/nano_gicp_registration.hpp"
#include "a2w_fastlio_common/quatro_registration.hpp"
#include "a2w_fastlio_common/scan_context_index.hpp"
#include "a2w_fastlio_common/scan_context_place_recognition.hpp"

namespace a2w_fastlio_common
{

AlgorithmSuite createDefaultAlgorithmSuite(const DefaultAlgorithmSuiteConfig & config)
{
  const auto place_recognition = std::make_shared<ScanContextPlaceRecognition>(
    ScanContextConfig{
      config.place_recognition.rings,
      config.place_recognition.sectors,
      config.place_recognition.max_radius_m,
      config.place_recognition.sensor_height_m});
  const auto descriptor_index = std::make_shared<ScanContextIndex>();
  const auto coarse = std::make_shared<QuatroRegistration>(
    QuatroRegistrationConfig{
      config.coarse.normal_radius_m,
      config.coarse.feature_radius_m,
      config.coarse.noise_bound_m,
      config.coarse.rotation_gnc_factor,
      config.coarse.rotation_cost_threshold,
      config.coarse.rotation_max_iterations,
      config.coarse.estimate_scale,
      config.coarse.optimized_matching,
      config.coarse.descriptor_distance_threshold,
      config.coarse.maximum_correspondences,
      config.coarse.minimum_points});
  const auto fine = std::make_shared<NanoGicpRegistration>(
    NanoGicpRegistrationConfig{
      config.fine.maximum_correspondence_distance_m,
      config.fine.thread_count,
      config.fine.correspondence_randomness,
      config.fine.maximum_iterations,
      config.fine.transformation_epsilon,
      config.fine.rotation_epsilon,
      config.fine.regularization_method,
      config.fine.fitness_score_max_range_m,
      config.fine.minimum_points});
  const MatchValidator validator{
    MatchValidationConfig{
      config.validation.maximum_fitness,
      config.validation.minimum_overlap,
      config.validation.minimum_correspondences,
      config.validation.maximum_translation_jump_m,
      config.validation.maximum_rotation_jump_rad,
      config.validation.minimum_candidate_distance_separation}};
  const auto registration = std::make_shared<RegistrationPipeline>(
    coarse, fine, validator, RegistrationPipelineConfig{config.evidence_distance_m});
  return {place_recognition, descriptor_index, registration};
}

}  // namespace a2w_fastlio_common
