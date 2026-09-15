#include "a2w_fastlio_localization/global_relocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

namespace a2w_fastlio_localization
{
namespace
{

using a2w_fastlio_common::Pose3d;

void validate(const RelocalizationConfig & config)
{
  const auto finite_nonnegative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (config.top_k < 2U || !finite_nonnegative(config.maximum_descriptor_distance) ||
    !finite_nonnegative(config.minimum_score_margin) ||
    !finite_nonnegative(config.descriptor_score_weight) ||
    !finite_nonnegative(config.fitness_score_weight) ||
    !finite_nonnegative(config.overlap_score_weight) ||
    config.fitness_score_weight + config.overlap_score_weight <= 0.0 ||
    !finite_nonnegative(config.strong_maximum_fitness) ||
    !std::isfinite(config.strong_minimum_overlap) || config.strong_minimum_overlap < 0.0 ||
    config.strong_minimum_overlap > 1.0 || config.strong_minimum_correspondences == 0U)
  {
    throw std::invalid_argument{"invalid global relocalization configuration"};
  }
}

bool strong(const CandidateAudit & audit, const RelocalizationConfig & config)
{
  return audit.registration.fitness <= config.strong_maximum_fitness &&
         audit.registration.overlap >= config.strong_minimum_overlap &&
         audit.registration.correspondence_count >= config.strong_minimum_correspondences;
}

}  // namespace

GlobalRelocalizer::GlobalRelocalizer(
  std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index,
  std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes,
  std::shared_ptr<const a2w_fastlio_common::CoarseRegistration> coarse,
  std::shared_ptr<const a2w_fastlio_common::FineRegistration> fine,
  a2w_fastlio_common::MatchValidator validator,
  a2w_fastlio_common::LocalMapConfig local_map_config,
  a2w_fastlio_common::RegistrationPipelineConfig pipeline_config)
: descriptor_index_{std::move(descriptor_index)}, keyframes_{std::move(keyframes)},
  local_map_builder_{local_map_config},
  registration_{std::make_shared<a2w_fastlio_common::RegistrationPipeline>(
      std::move(coarse), std::move(fine), std::move(validator), pipeline_config)}
{
  if (!descriptor_index_ || !keyframes_) {
    throw std::invalid_argument{"global relocalizer requires descriptor and keyframe interfaces"};
  }
}

GlobalRelocalizer::GlobalRelocalizer(
  std::shared_ptr<const a2w_fastlio_common::DescriptorIndex> descriptor_index,
  std::shared_ptr<const a2w_fastlio_common::KeyFrameProvider> keyframes,
  std::shared_ptr<const a2w_fastlio_common::RegistrationPipeline> registration,
  a2w_fastlio_common::LocalMapConfig local_map_config)
: descriptor_index_{std::move(descriptor_index)}, keyframes_{std::move(keyframes)},
  local_map_builder_{local_map_config}, registration_{std::move(registration)}
{
  if (!descriptor_index_ || !keyframes_ || !registration_) {
    throw std::invalid_argument{"global relocalizer requires common algorithm interfaces"};
  }
}

RelocalizationResult GlobalRelocalizer::evaluate(
  const a2w_fastlio_common::CloudConstPtr & current_local_map,
  const a2w_fastlio_common::ScanDescriptor & query_descriptor,
  const RelocalizationConfig & config) const
{
  validate(config);
  if (!current_local_map || current_local_map->empty()) {
    throw std::invalid_argument{"global relocalization query cloud must not be empty"};
  }
  RelocalizationResult result;
  const auto candidates = descriptor_index_->queryTopK(
    query_descriptor, config.top_k, a2w_fastlio_common::CandidateFilter{});
  std::vector<std::size_t> accepted;
  result.audits.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    CandidateAudit audit;
    audit.candidate = candidate;
    if (!std::isfinite(candidate.distance) || candidate.distance < 0.0 ||
      candidate.distance > config.maximum_descriptor_distance)
    {
      audit.reason = "descriptor_distance_above_limit";
      result.audits.push_back(std::move(audit));
      continue;
    }
    try {
      const auto center = keyframes_->get(candidate.keyframe_id);
      if (!center) {
        throw std::runtime_error{"candidate keyframe is missing"};
      }
      const auto target = local_map_builder_.build(
        candidate.keyframe_id, config.neighbor_keyframes_before,
        config.neighbor_keyframes_after, *keyframes_);
      Pose3d hint = center->optimized_pose;
      hint.rotation = (hint.rotation * Eigen::Quaterniond{
        Eigen::AngleAxisd{candidate.yaw_hint_rad, Eigen::Vector3d::UnitZ()}}).normalized();
      const auto pipeline = registration_->run(current_local_map, target.cloud, hint, {});
      audit.evaluated = true;
      audit.accepted = pipeline.success;
      audit.reason = pipeline.validation.reason;
      audit.registration = pipeline.fine;
      audit.map_body = pipeline.final_transform;
      if (audit.accepted) {
        audit.score = config.descriptor_score_weight * candidate.distance +
          config.fitness_score_weight * audit.registration.fitness +
          config.overlap_score_weight * (1.0 - audit.registration.overlap);
      }
    } catch (const std::exception &) {
      audit.reason = "candidate_local_map_invalid";
    }
    result.audits.push_back(std::move(audit));
    if (result.audits.back().accepted) {
      accepted.push_back(result.audits.size() - 1U);
    }
  }
  if (accepted.empty()) {
    result.reason = candidates.empty() ? "no_candidates" : "no_geometrically_valid_candidates";
    return result;
  }
  std::stable_sort(accepted.begin(), accepted.end(), [&result](const auto lhs, const auto rhs) {
      const auto & left = result.audits[lhs];
      const auto & right = result.audits[rhs];
      return left.score != right.score ? left.score < right.score :
             left.candidate.keyframe_id < right.candidate.keyframe_id;
    });
  const auto & best = result.audits[accepted.front()];
  result.best_score = best.score;
  if (accepted.size() > 1U) {
    result.second_best_score = result.audits[accepted[1U]].score;
    if (result.second_best_score - result.best_score < config.minimum_score_margin) {
      result.reason = "candidate_score_ambiguous";
      return result;
    }
  }
  result.success = true;
  result.strong = strong(best, config);
  result.reason = result.strong ? "strong_candidate_accepted" : "candidate_accepted";
  result.candidate_id = best.candidate.keyframe_id;
  result.map_body = best.map_body;
  result.registration = best.registration;
  return result;
}

RelocalizationSession::RelocalizationSession(RelocalizationSessionConfig config) : config_{config}
{
  if (config_.confirmation_count < 2U ||
    !std::isfinite(config_.maximum_translation_difference_m) ||
    config_.maximum_translation_difference_m < 0.0 ||
    !std::isfinite(config_.maximum_rotation_difference_rad) ||
    config_.maximum_rotation_difference_rad < 0.0 ||
    config_.maximum_rotation_difference_rad > M_PI || config_.timeout_ns <= 0)
  {
    throw std::invalid_argument{"invalid relocalization session configuration"};
  }
}

void RelocalizationSession::requireMonotonic(const std::int64_t stamp_ns) const
{
  if (stamp_ns < 0 || (last_stamp_ns_ && stamp_ns <= *last_stamp_ns_)) {
    throw std::invalid_argument{"relocalization session timestamps must be increasing"};
  }
}

RelocalizationSessionSnapshot RelocalizationSession::start(const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  last_stamp_ns_ = stamp_ns;
  started_ns_ = stamp_ns;
  previous_correction_.reset();
  snapshot_ = RelocalizationSessionSnapshot{};
  snapshot_.active = true;
  snapshot_.reason = "started";
  snapshot_.stamp_ns = stamp_ns;
  return snapshot_;
}

RelocalizationSessionSnapshot RelocalizationSession::observe(
  const std::int64_t stamp_ns, const RelocalizationResult & result)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  last_stamp_ns_ = stamp_ns;
  snapshot_.stamp_ns = stamp_ns;
  snapshot_.confirmed_correction.reset();
  if (!snapshot_.active || !started_ns_) {
    throw std::logic_error{"cannot observe an inactive relocalization session"};
  }
  if (stamp_ns - *started_ns_ >= config_.timeout_ns) {
    snapshot_.active = false;
    snapshot_.consecutive_matches = 0U;
    snapshot_.reason = "session_timeout";
    previous_correction_.reset();
    return snapshot_;
  }
  if (!result.success || !a2w_fastlio_common::isFinitePose(result.map_body)) {
    snapshot_.consecutive_matches = 0U;
    snapshot_.reason = result.reason.empty() ? "candidate_rejected" : result.reason;
    previous_correction_.reset();
    return snapshot_;
  }
  bool consistent = previous_correction_ && snapshot_.candidate_id == result.candidate_id;
  if (consistent) {
    const double translation =
      (previous_correction_->translation - result.map_body.translation).norm();
    const double rotation = Eigen::AngleAxisd{
      previous_correction_->rotation.inverse() * result.map_body.rotation}.angle();
    consistent = std::isfinite(rotation) &&
      translation <= config_.maximum_translation_difference_m &&
      rotation <= config_.maximum_rotation_difference_rad;
  }
  snapshot_.candidate_id = result.candidate_id;
  snapshot_.consecutive_matches = consistent ? snapshot_.consecutive_matches + 1U : 1U;
  previous_correction_ = result.map_body;
  snapshot_.reason = consistent ? "confirmation_pending" : "confirmation_restarted";
  if (result.strong || snapshot_.consecutive_matches >= config_.confirmation_count) {
    snapshot_.active = false;
    snapshot_.reason = result.strong ? "strong_result_confirmed" : "multiframe_confirmed";
    snapshot_.confirmed_correction = result.map_body;
  }
  return snapshot_;
}

RelocalizationSessionSnapshot RelocalizationSession::cancel(const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  last_stamp_ns_ = stamp_ns;
  snapshot_.stamp_ns = stamp_ns;
  snapshot_.active = false;
  snapshot_.reason = "cancelled";
  snapshot_.consecutive_matches = 0U;
  snapshot_.confirmed_correction.reset();
  previous_correction_.reset();
  started_ns_.reset();
  return snapshot_;
}

RelocalizationSessionSnapshot RelocalizationSession::latest() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return snapshot_;
}

}  // namespace a2w_fastlio_localization
