#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "a2w_fastlio_common/local_map_builder.hpp"
#include "a2w_fastlio_common/place_recognition.hpp"
#include "a2w_fastlio_common/registration_pipeline.hpp"

namespace a2w_fastlio_localization
{

struct RelocalizationConfig
{
  std::size_t top_k{5U};
  std::size_t neighbor_keyframes_before{2U};
  std::size_t neighbor_keyframes_after{2U};
  double maximum_descriptor_distance{0.5};
  double minimum_score_margin{0.05};
  double descriptor_score_weight{0.1};
  double fitness_score_weight{1.0};
  double overlap_score_weight{1.0};
  double strong_maximum_fitness{0.08};
  double strong_minimum_overlap{0.7};
  std::size_t strong_minimum_correspondences{100U};
};

struct CandidateAudit
{
  a2w_fastlio_common::LoopCandidate candidate{};
  bool evaluated{false};
  bool accepted{false};
  std::string reason{};
  double score{std::numeric_limits<double>::infinity()};
  a2w_fastlio_common::RegistrationResult registration{};
  a2w_fastlio_common::Pose3d map_body{};
};

struct RelocalizationResult
{
  bool success{false};
  bool strong{false};
  std::string reason{"no_candidates"};
  std::uint64_t candidate_id{0U};
  double best_score{std::numeric_limits<double>::infinity()};
  double second_best_score{std::numeric_limits<double>::infinity()};
  a2w_fastlio_common::Pose3d map_body{};
  a2w_fastlio_common::RegistrationResult registration{};
  std::vector<CandidateAudit> audits{};
};

class GlobalRelocalizer
{
public:
  GlobalRelocalizer(
    std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index,
    std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes,
    std::shared_ptr<const a2w_fastlio_common::CoarseRegistration> coarse,
    std::shared_ptr<const a2w_fastlio_common::FineRegistration> fine,
    a2w_fastlio_common::MatchValidator validator,
    a2w_fastlio_common::LocalMapConfig local_map_config,
    a2w_fastlio_common::RegistrationPipelineConfig pipeline_config);
  GlobalRelocalizer(
    std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index,
    std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes,
    std::shared_ptr<const a2w_fastlio_common::RegistrationPipeline> registration,
    a2w_fastlio_common::LocalMapConfig local_map_config);

  RelocalizationResult evaluate(
    const a2w_fastlio_common::CloudConstPtr & current_local_map,
    const a2w_fastlio_common::ScanDescriptor & query_descriptor,
    const RelocalizationConfig & config) const;

private:
  std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index_;
  std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes_;
  a2w_fastlio_common::LocalMapBuilder local_map_builder_;
  std::shared_ptr<const a2w_fastlio_common::RegistrationPipeline> registration_;
};

struct RelocalizationSessionConfig
{
  std::size_t confirmation_count{3U};
  double maximum_translation_difference_m{1.0};
  double maximum_rotation_difference_rad{0.25};
  std::int64_t timeout_ns{10'000'000'000LL};
};

struct RelocalizationSessionSnapshot
{
  bool active{false};
  std::string reason{"idle"};
  std::int64_t stamp_ns{0};
  std::uint64_t candidate_id{0U};
  std::size_t consecutive_matches{0U};
  std::optional<a2w_fastlio_common::Pose3d> confirmed_correction{};
};

class RelocalizationSession
{
public:
  explicit RelocalizationSession(RelocalizationSessionConfig config);
  RelocalizationSessionSnapshot start(std::int64_t stamp_ns);
  RelocalizationSessionSnapshot observe(
    std::int64_t stamp_ns, const RelocalizationResult & result);
  RelocalizationSessionSnapshot cancel(std::int64_t stamp_ns);
  RelocalizationSessionSnapshot latest() const;

private:
  void requireMonotonic(std::int64_t stamp_ns) const;

  RelocalizationSessionConfig config_;
  mutable std::mutex mutex_;
  RelocalizationSessionSnapshot snapshot_;
  std::optional<std::int64_t> last_stamp_ns_;
  std::optional<std::int64_t> started_ns_;
  std::optional<a2w_fastlio_common::Pose3d> previous_correction_;
};

}  // namespace a2w_fastlio_localization
