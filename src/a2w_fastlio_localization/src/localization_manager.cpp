#include "a2w_fastlio_localization/localization_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_common::Pose3d compose(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = (lhs.rotation * rhs.rotation).normalized();
  result.translation = lhs.rotation * rhs.translation + lhs.translation;
  return result;
}

a2w_fastlio_common::Pose3d inverse(const a2w_fastlio_common::Pose3d & pose)
{
  a2w_fastlio_common::Pose3d result;
  result.rotation = pose.rotation.normalized().conjugate();
  result.translation = -(result.rotation * pose.translation);
  return result;
}

double rotationDistance(
  const a2w_fastlio_common::Pose3d & lhs, const a2w_fastlio_common::Pose3d & rhs)
{
  return Eigen::AngleAxisd(lhs.rotation.conjugate() * rhs.rotation).angle();
}

}  // namespace

LocalizationManager::LocalizationManager(LocalizationManagerConfig config, MatchFunction matcher)
: config_{config}, matcher_{std::move(matcher)}
{
  if (config_.match_interval_ns <= 0 ||
    !std::isfinite(config_.maximum_correction_translation_jump_m) ||
    config_.maximum_correction_translation_jump_m <= 0.0 ||
    !std::isfinite(config_.maximum_correction_rotation_jump_rad) ||
    config_.maximum_correction_rotation_jump_rad <= 0.0 || !matcher_)
  {
    throw std::invalid_argument{"invalid LocalizationManager configuration"};
  }
}

LocalizationOutput LocalizationManager::process(const a2w_fastlio_common::FrontendFrame & frame)
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (frame.stamp_ns < 0 || (last_frame_ns_ && frame.stamp_ns <= *last_frame_ns_) ||
    !a2w_fastlio_common::isFinitePose(frame.odom_pose))
  {
    throw std::invalid_argument{"Localization frame is invalid or non-monotonic"};
  }
  last_frame_ns_ = frame.stamp_ns;
  LocalizationOutput output;
  output.stamp_ns = frame.stamp_ns;
  const auto predicted = correction_ ? compose(*correction_, frame.odom_pose) : frame.odom_pose;
  const bool should_match = !last_attempt_ns_ ||
    frame.stamp_ns - *last_attempt_ns_ >= config_.match_interval_ns;
  if (should_match) {
    output.match_attempted = true;
    last_attempt_ns_ = frame.stamp_ns;
    const auto match = matcher_(frame, predicted);
    output.candidate_id = match.candidate_id;
    output.registration = match.registration;
    if (match.success && a2w_fastlio_common::isFinitePose(match.map_body)) {
      const auto candidate = compose(match.map_body, inverse(frame.odom_pose));
      const bool jump = correction_ &&
        ((candidate.translation - correction_->translation).norm() >
        config_.maximum_correction_translation_jump_m ||
        rotationDistance(*correction_, candidate) > config_.maximum_correction_rotation_jump_rad);
      if (!jump) {
        correction_ = candidate;
        correction_stamp_ns_ = frame.stamp_ns;
        output.match_accepted = true;
        output.reason = "match_accepted";
      } else {
        output.reason = "correction_jump_rejected";
      }
    } else {
      output.reason = match.reason.empty() ? "match_failed" : match.reason;
    }
  } else {
    output.reason = "continuous_from_last_correction";
  }
  if (correction_) {
    output.valid = true;
    output.map_camera_init = *correction_;
    output.map_body = compose(*correction_, frame.odom_pose);
    output.correction_stamp_ns = *correction_stamp_ns_;
    output.correction_age_ns = frame.stamp_ns - *correction_stamp_ns_;
  }
  latest_ = output;
  return output;
}

std::optional<LocalizationOutput> LocalizationManager::latest() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return latest_;
}

}  // namespace a2w_fastlio_localization
