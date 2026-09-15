#pragma once

#include <cstddef>
#include <memory>

#include "a2w_fastlio_common/place_recognition.hpp"
#include "a2w_fastlio_common/registration_pipeline.hpp"

namespace a2w_fastlio_common
{

struct PlaceRecognitionParameters
{
  std::size_t rings{20U};
  std::size_t sectors{60U};
  double max_radius_m{80.0};
  double sensor_height_m{2.0};
};

struct CoarseRegistrationParameters
{
  double normal_radius_m{0.5};
  double feature_radius_m{0.8};
  double noise_bound_m{0.25};
  double rotation_gnc_factor{1.4};
  double rotation_cost_threshold{1e-4};
  int rotation_max_iterations{100};
  bool estimate_scale{false};
  bool optimized_matching{true};
  double descriptor_distance_threshold{30.0};
  int maximum_correspondences{500};
  std::size_t minimum_points{20U};
};

struct FineRegistrationParameters
{
  double maximum_correspondence_distance_m{2.0};
  int thread_count{0};
  int correspondence_randomness{20};
  int maximum_iterations{64};
  double transformation_epsilon{1e-3};
  double rotation_epsilon{1e-3};
  int regularization_method{3};
  double fitness_score_max_range_m{1.0};
  std::size_t minimum_points{20U};
};

struct MatchValidationParameters
{
  double maximum_fitness{0.25};
  double minimum_overlap{0.35};
  std::size_t minimum_correspondences{30U};
  double maximum_translation_jump_m{5.0};
  double maximum_rotation_jump_rad{1.0};
  double minimum_candidate_distance_separation{0.05};
};

struct DefaultAlgorithmSuiteConfig
{
  PlaceRecognitionParameters place_recognition{};
  CoarseRegistrationParameters coarse{};
  FineRegistrationParameters fine{};
  MatchValidationParameters validation{};
  double evidence_distance_m{0.5};
};

struct AlgorithmSuite
{
  std::shared_ptr<PlaceRecognition> place_recognition;
  std::shared_ptr<DescriptorIndex> descriptor_index;
  std::shared_ptr<RegistrationPipeline> registration;
};

AlgorithmSuite createDefaultAlgorithmSuite(const DefaultAlgorithmSuiteConfig & config);

}  // namespace a2w_fastlio_common
