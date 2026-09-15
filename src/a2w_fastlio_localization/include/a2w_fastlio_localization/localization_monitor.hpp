#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_localization
{

enum class LocalizationState : std::uint8_t
{
  kInitializing = 0U,
  kLocalized = 1U,
  kDegraded = 2U,
  kLost = 3U,
  kRelocalizing = 4U,
};

const char * localizationStateName(LocalizationState state) noexcept;

enum class EvidenceSource : std::uint8_t
{
  kNormal = 0U,
  kRelocalization = 1U,
};

struct MatchEvidence
{
  std::int64_t stamp_ns{0};
  bool match_attempted{false};
  bool accepted{false};
  bool correction_available{false};
  std::int64_t correction_age_ns{0};
  EvidenceSource source{EvidenceSource::kNormal};
  std::uint64_t candidate_id{0U};
  a2w_fastlio_common::RegistrationResult registration{};
  bool defer_confirmation{false};
  bool confirmation_complete{false};
};

struct LocalizationMonitorConfig
{
  std::size_t initialization_successes_required{2U};
  std::size_t degraded_failures_required{2U};
  std::size_t lost_failures_required{5U};
  std::size_t normal_recovery_successes_required{2U};
  std::size_t relocalization_successes_required{3U};
  std::int64_t correction_stale_after_ns{3'000'000'000LL};
  std::int64_t relocalization_timeout_ns{10'000'000'000LL};
  double strong_maximum_fitness{0.08};
  double strong_minimum_overlap{0.7};
  std::size_t strong_minimum_correspondences{100U};
};

struct LocalizationStatusSnapshot
{
  LocalizationState state{LocalizationState::kInitializing};
  std::string reason{"awaiting_initial_match"};
  std::int64_t stamp_ns{0};
  std::size_t consecutive_successes{0U};
  std::size_t consecutive_failures{0U};
  std::size_t relocalization_successes{0U};
  bool correction_available{false};
  std::int64_t correction_age_ns{0};
  std::uint64_t candidate_id{0U};
  a2w_fastlio_common::RegistrationResult registration{};
  bool hardware_validation_pending{true};
};

class LocalizationMonitor
{
public:
  explicit LocalizationMonitor(LocalizationMonitorConfig config);

  LocalizationStatusSnapshot update(const MatchEvidence & evidence);
  LocalizationStatusSnapshot beginRelocalization(std::int64_t stamp_ns);
  LocalizationStatusSnapshot reset(std::int64_t stamp_ns);
  LocalizationStatusSnapshot latest() const;

private:
  void requireMonotonic(std::int64_t stamp_ns) const;
  void recordTimestamp(std::int64_t stamp_ns);
  bool isStrongRelocalization(const MatchEvidence & evidence) const noexcept;
  void transition(LocalizationState state, const char * reason);

  LocalizationMonitorConfig config_;
  mutable std::mutex mutex_;
  LocalizationStatusSnapshot snapshot_;
  std::optional<std::int64_t> last_stamp_ns_;
  std::optional<std::int64_t> relocalization_started_ns_;
};

}  // namespace a2w_fastlio_localization
