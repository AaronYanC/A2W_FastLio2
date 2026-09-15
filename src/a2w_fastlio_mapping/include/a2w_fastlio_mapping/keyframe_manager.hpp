#pragma once

#include <cstddef>
#include <optional>

#include "a2w_fastlio_common/types.hpp"

namespace a2w_fastlio_mapping
{

struct KeyframeConfig
{
  double translation_threshold_m{1.0};
  double rotation_threshold_rad{0.17453292519943295};
  double max_interval_s{2.0};
};

enum class KeyframeDecisionReason
{
  kAcceptedFirst,
  kAcceptedTranslation,
  kAcceptedRotation,
  kAcceptedMaxInterval,
  kRejectedBelowThreshold,
  kRejectedNonMonotonicTimestamp,
  kRejectedInvalidPose,
  kRejectedEmptyCloud,
};

struct KeyframeDecision
{
  KeyframeDecisionReason reason{KeyframeDecisionReason::kRejectedBelowThreshold};
  std::optional<a2w_fastlio_common::KeyFrame> keyframe{};
};

class KeyframeManager
{
public:
  explicit KeyframeManager(KeyframeConfig config);

  KeyframeDecision consider(const a2w_fastlio_common::FrontendFrame & frame);
  std::size_t size() const noexcept;
  void reset() noexcept;

private:
  a2w_fastlio_common::KeyFrame makeKeyframe(
    const a2w_fastlio_common::FrontendFrame & frame) const;

  KeyframeConfig config_;
  std::optional<a2w_fastlio_common::KeyFrame> last_keyframe_{};
  std::size_t size_{0};
};

}  // namespace a2w_fastlio_mapping
