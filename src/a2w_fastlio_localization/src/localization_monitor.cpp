#include "a2w_fastlio_localization/localization_monitor.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace a2w_fastlio_localization
{

const char * localizationStateName(const LocalizationState state) noexcept
{
  switch (state) {
    case LocalizationState::kInitializing:
      return "INITIALIZING";
    case LocalizationState::kLocalized:
      return "LOCALIZED";
    case LocalizationState::kDegraded:
      return "DEGRADED";
    case LocalizationState::kLost:
      return "LOST";
    case LocalizationState::kRelocalizing:
      return "RELOCALIZING";
  }
  return "UNKNOWN";
}

LocalizationMonitor::LocalizationMonitor(LocalizationMonitorConfig config)
: config_(std::move(config))
{
  if (config_.initialization_successes_required == 0U ||
    config_.degraded_failures_required == 0U || config_.lost_failures_required == 0U ||
    config_.normal_recovery_successes_required == 0U ||
    config_.relocalization_successes_required == 0U ||
    config_.lost_failures_required < config_.degraded_failures_required ||
    config_.correction_stale_after_ns <= 0 || config_.relocalization_timeout_ns <= 0 ||
    !std::isfinite(config_.strong_maximum_fitness) || config_.strong_maximum_fitness < 0.0 ||
    !std::isfinite(config_.strong_minimum_overlap) ||
    config_.strong_minimum_overlap < 0.0 || config_.strong_minimum_overlap > 1.0 ||
    config_.strong_minimum_correspondences == 0U)
  {
    throw std::invalid_argument{"invalid localization monitor configuration"};
  }
}

void LocalizationMonitor::requireMonotonic(const std::int64_t stamp_ns) const
{
  if (stamp_ns < 0 || (last_stamp_ns_ && stamp_ns <= *last_stamp_ns_)) {
    throw std::invalid_argument{"localization monitor timestamps must be non-negative and increasing"};
  }
}

void LocalizationMonitor::recordTimestamp(const std::int64_t stamp_ns)
{
  last_stamp_ns_ = stamp_ns;
  snapshot_.stamp_ns = stamp_ns;
}

void LocalizationMonitor::transition(const LocalizationState state, const char * reason)
{
  snapshot_.state = state;
  snapshot_.reason = reason;
}

bool LocalizationMonitor::isStrongRelocalization(const MatchEvidence & evidence) const noexcept
{
  return evidence.accepted && std::isfinite(evidence.registration.fitness) &&
         evidence.registration.fitness <= config_.strong_maximum_fitness &&
         std::isfinite(evidence.registration.overlap) &&
         evidence.registration.overlap >= config_.strong_minimum_overlap &&
         evidence.registration.correspondence_count >= config_.strong_minimum_correspondences;
}

LocalizationStatusSnapshot LocalizationMonitor::update(const MatchEvidence & evidence)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(evidence.stamp_ns);
  recordTimestamp(evidence.stamp_ns);
  snapshot_.correction_available = evidence.correction_available;
  snapshot_.correction_age_ns = evidence.correction_age_ns;
  snapshot_.candidate_id = evidence.candidate_id;
  snapshot_.registration = evidence.registration;

  if ((snapshot_.state == LocalizationState::kLocalized ||
    snapshot_.state == LocalizationState::kDegraded) && evidence.correction_available &&
    evidence.correction_age_ns >= config_.correction_stale_after_ns)
  {
    snapshot_.consecutive_successes = 0U;
    transition(LocalizationState::kLost, "correction_stale");
    return snapshot_;
  }

  if (snapshot_.state == LocalizationState::kLost) {
    return snapshot_;
  }

  if (snapshot_.state == LocalizationState::kRelocalizing) {
    if (!relocalization_started_ns_) {
      throw std::logic_error{"relocalization start timestamp is missing"};
    }
    if (evidence.stamp_ns - *relocalization_started_ns_ >= config_.relocalization_timeout_ns) {
      snapshot_.relocalization_successes = 0U;
      transition(LocalizationState::kLost, "relocalization_timeout");
      return snapshot_;
    }
    if (!evidence.match_attempted || evidence.source != EvidenceSource::kRelocalization) {
      transition(LocalizationState::kRelocalizing, "awaiting_relocalization_match");
      return snapshot_;
    }
    if (!evidence.accepted) {
      snapshot_.relocalization_successes = 0U;
      transition(LocalizationState::kRelocalizing, "relocalization_rejected");
      return snapshot_;
    }
    ++snapshot_.relocalization_successes;
    if (isStrongRelocalization(evidence) ||
      snapshot_.relocalization_successes >= config_.relocalization_successes_required)
    {
      snapshot_.consecutive_successes = 0U;
      snapshot_.consecutive_failures = 0U;
      snapshot_.relocalization_successes = 0U;
      relocalization_started_ns_.reset();
      transition(LocalizationState::kLocalized, "relocalization_accepted");
    } else {
      transition(LocalizationState::kRelocalizing, "relocalization_confirmation_pending");
    }
    return snapshot_;
  }

  if (!evidence.match_attempted || evidence.source != EvidenceSource::kNormal) {
    snapshot_.reason = "no_match_attempt";
    return snapshot_;
  }

  if (evidence.accepted) {
    ++snapshot_.consecutive_successes;
    snapshot_.consecutive_failures = 0U;
    if (snapshot_.state == LocalizationState::kInitializing &&
      snapshot_.consecutive_successes >= config_.initialization_successes_required)
    {
      snapshot_.consecutive_successes = 0U;
      transition(LocalizationState::kLocalized, "initialization_confirmed");
    } else if (snapshot_.state == LocalizationState::kDegraded &&
      snapshot_.consecutive_successes >= config_.normal_recovery_successes_required)
    {
      snapshot_.consecutive_successes = 0U;
      transition(LocalizationState::kLocalized, "normal_tracking_recovered");
    } else {
      snapshot_.reason = snapshot_.state == LocalizationState::kInitializing ?
        "initialization_confirmation_pending" : "tracking_accepted";
    }
    return snapshot_;
  }

  snapshot_.consecutive_successes = 0U;
  ++snapshot_.consecutive_failures;
  if (snapshot_.state == LocalizationState::kInitializing) {
    snapshot_.reason = "initial_match_rejected";
  } else if (snapshot_.consecutive_failures >= config_.lost_failures_required) {
    transition(LocalizationState::kLost, "consecutive_match_failures");
  } else if (snapshot_.consecutive_failures >= config_.degraded_failures_required) {
    transition(LocalizationState::kDegraded, "consecutive_match_failures");
  } else {
    snapshot_.reason = "match_rejected";
  }
  return snapshot_;
}

LocalizationStatusSnapshot LocalizationMonitor::beginRelocalization(const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  recordTimestamp(stamp_ns);
  if (snapshot_.state != LocalizationState::kLost) {
    throw std::logic_error{"relocalization may only begin from LOST"};
  }
  snapshot_.consecutive_successes = 0U;
  snapshot_.relocalization_successes = 0U;
  relocalization_started_ns_ = stamp_ns;
  transition(LocalizationState::kRelocalizing, "relocalization_started");
  return snapshot_;
}

LocalizationStatusSnapshot LocalizationMonitor::reset(const std::int64_t stamp_ns)
{
  std::lock_guard<std::mutex> lock{mutex_};
  requireMonotonic(stamp_ns);
  snapshot_ = LocalizationStatusSnapshot{};
  recordTimestamp(stamp_ns);
  snapshot_.reason = "reset";
  relocalization_started_ns_.reset();
  return snapshot_;
}

LocalizationStatusSnapshot LocalizationMonitor::latest() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return snapshot_;
}

}  // namespace a2w_fastlio_localization
