#include "a2w_fastlio_mapping/keyframe_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_mapping
{

namespace
{

constexpr double kComparisonTolerance = 1.0e-12;

double rotationDistance(
  const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs)
{
  const double dot = std::clamp(std::abs(lhs.dot(rhs)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

bool reached(const double value, const double threshold)
{
  return value + kComparisonTolerance >= threshold;
}

bool positiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool validPose(const a2w_fastlio_common::Pose3d & pose)
{
  return pose.translation.allFinite() && pose.rotation.coeffs().allFinite() &&
         pose.rotation.squaredNorm() > kComparisonTolerance;
}

}  // namespace

KeyframeManager::KeyframeManager(KeyframeConfig config)
: config_(std::move(config))
{
  if (!positiveFinite(config_.translation_threshold_m) ||
    !positiveFinite(config_.rotation_threshold_rad) ||
    !positiveFinite(config_.max_interval_s))
  {
    throw std::invalid_argument("keyframe thresholds must be finite and greater than zero");
  }
}

KeyframeDecision KeyframeManager::consider(
  const a2w_fastlio_common::FrontendFrame & frame)
{
  if (!frame.body_cloud || frame.body_cloud->empty()) {
    return {KeyframeDecisionReason::kRejectedEmptyCloud, std::nullopt};
  }
  if (!validPose(frame.odom_pose)) {
    return {KeyframeDecisionReason::kRejectedInvalidPose, std::nullopt};
  }
  if (last_keyframe_ && frame.stamp_ns <= last_keyframe_->stamp_ns) {
    return {KeyframeDecisionReason::kRejectedNonMonotonicTimestamp, std::nullopt};
  }

  if (!last_keyframe_) {
    auto keyframe = makeKeyframe(frame);
    last_keyframe_ = keyframe;
    ++size_;
    return {KeyframeDecisionReason::kAcceptedFirst, std::move(keyframe)};
  }

  const double translation_delta =
    (frame.odom_pose.translation - last_keyframe_->odom_pose.translation).norm();
  const double rotation_delta = rotationDistance(
    frame.odom_pose.rotation, last_keyframe_->odom_pose.rotation);
  const double elapsed_s =
    static_cast<double>(frame.stamp_ns - last_keyframe_->stamp_ns) / 1.0e9;

  KeyframeDecisionReason reason;
  if (reached(translation_delta, config_.translation_threshold_m)) {
    reason = KeyframeDecisionReason::kAcceptedTranslation;
  } else if (reached(rotation_delta, config_.rotation_threshold_rad)) {
    reason = KeyframeDecisionReason::kAcceptedRotation;
  } else if (reached(elapsed_s, config_.max_interval_s)) {
    reason = KeyframeDecisionReason::kAcceptedMaxInterval;
  } else {
    return {KeyframeDecisionReason::kRejectedBelowThreshold, std::nullopt};
  }

  auto keyframe = makeKeyframe(frame);
  last_keyframe_ = keyframe;
  ++size_;
  return {reason, std::move(keyframe)};
}

std::size_t KeyframeManager::size() const noexcept
{
  return size_;
}

void KeyframeManager::reset() noexcept
{
  last_keyframe_.reset();
  size_ = 0;
}

a2w_fastlio_common::KeyFrame KeyframeManager::makeKeyframe(
  const a2w_fastlio_common::FrontendFrame & frame) const
{
  a2w_fastlio_common::KeyFrame keyframe;
  keyframe.id = size_;
  keyframe.stamp_ns = frame.stamp_ns;
  keyframe.odom_pose = frame.odom_pose;
  keyframe.odom_pose.rotation.normalize();
  keyframe.optimized_pose = frame.odom_pose;
  keyframe.optimized_pose.rotation.normalize();
  keyframe.body_cloud.reset(new a2w_fastlio_common::Cloud{*frame.body_cloud});
  return keyframe;
}

}  // namespace a2w_fastlio_mapping
