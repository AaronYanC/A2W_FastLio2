#include <limits>

#include <gtest/gtest.h>

#include "a2w_fastlio_localization/localization_monitor.hpp"

namespace a2w_fastlio_localization
{
namespace
{

MatchEvidence evidence(
  const std::int64_t stamp, const bool accepted,
  const EvidenceSource source = EvidenceSource::kNormal)
{
  MatchEvidence result;
  result.stamp_ns = stamp;
  result.match_attempted = true;
  result.accepted = accepted;
  result.correction_available = accepted;
  result.source = source;
  result.candidate_id = 7U;
  result.registration.fitness = accepted ? 0.05 : 1.0;
  result.registration.overlap = accepted ? 0.8 : 0.0;
  result.registration.correspondence_count = accepted ? 100U : 0U;
  return result;
}

LocalizationMonitorConfig config()
{
  LocalizationMonitorConfig result;
  result.initialization_successes_required = 2U;
  result.degraded_failures_required = 2U;
  result.lost_failures_required = 4U;
  result.normal_recovery_successes_required = 2U;
  result.relocalization_successes_required = 3U;
  result.correction_stale_after_ns = 100;
  result.relocalization_timeout_ns = 500;
  result.strong_maximum_fitness = 0.02;
  result.strong_minimum_overlap = 0.9;
  result.strong_minimum_correspondences = 150U;
  return result;
}

TEST(LocalizationMonitor, CoversInitializationDegradationLossAndReset)
{
  LocalizationMonitor monitor{config()};
  EXPECT_EQ(monitor.latest().state, LocalizationState::kInitializing);
  EXPECT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kInitializing);
  EXPECT_EQ(monitor.update(evidence(20, true)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(30, false)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(40, false)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(50, true)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(60, true)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(70, false)).state, LocalizationState::kLocalized);
  EXPECT_EQ(monitor.update(evidence(80, false)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(90, false)).state, LocalizationState::kDegraded);
  EXPECT_EQ(monitor.update(evidence(100, false)).state, LocalizationState::kLost);
  const auto reset = monitor.reset(110);
  EXPECT_EQ(reset.state, LocalizationState::kInitializing);
  EXPECT_EQ(reset.reason, "reset");
}

TEST(LocalizationMonitor, StaleCorrectionForcesLostWithoutRefreshingOnFailure)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  LocalizationMonitor monitor{configured};
  EXPECT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);
  auto continuous = evidence(109, false);
  continuous.match_attempted = false;
  continuous.correction_available = true;
  continuous.correction_age_ns = 99;
  EXPECT_EQ(monitor.update(continuous).state, LocalizationState::kLocalized);
  continuous.stamp_ns = 110;
  continuous.correction_age_ns = 100;
  EXPECT_EQ(monitor.update(continuous).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.latest().reason, "correction_stale");
}

TEST(LocalizationMonitor, LostRequiresExplicitRelocalizationAndConsistentRecovery)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  ASSERT_EQ(monitor.update(evidence(10, true)).state, LocalizationState::kLocalized);
  ASSERT_EQ(monitor.update(evidence(20, false)).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.update(evidence(30, true)).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.latest().reason, "consecutive_match_failures");
  EXPECT_EQ(monitor.beginRelocalization(40).state, LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(50, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(60, false, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(70, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(80, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kRelocalizing);
  EXPECT_EQ(monitor.update(evidence(90, true, EvidenceSource::kRelocalization)).state,
    LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, StrongRelocalizationCanRecoverOnceAndTimeoutReturnsLost)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  monitor.update(evidence(20, false));
  monitor.beginRelocalization(30);
  auto strong = evidence(40, true, EvidenceSource::kRelocalization);
  strong.registration.fitness = 0.01;
  strong.registration.overlap = 0.95;
  strong.registration.correspondence_count = 200U;
  EXPECT_EQ(monitor.update(strong).state, LocalizationState::kLocalized);

  monitor.update(evidence(50, false));
  monitor.beginRelocalization(60);
  auto idle = evidence(560, false, EvidenceSource::kRelocalization);
  idle.match_attempted = false;
  EXPECT_EQ(monitor.update(idle).state, LocalizationState::kLost);
  EXPECT_EQ(monitor.latest().reason, "relocalization_timeout");
}

TEST(LocalizationMonitor, ExternalConsistencyPolicyGatesRecovery)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.lost_failures_required = 1U;
  configured.degraded_failures_required = 1U;
  LocalizationMonitor monitor{configured};
  monitor.update(evidence(10, true));
  monitor.update(evidence(20, false));
  monitor.beginRelocalization(30);
  auto pending = evidence(40, true, EvidenceSource::kRelocalization);
  pending.defer_confirmation = true;
  EXPECT_EQ(monitor.update(pending).state, LocalizationState::kRelocalizing);
  pending.stamp_ns = 50;
  EXPECT_EQ(monitor.update(pending).state, LocalizationState::kRelocalizing);
  pending.stamp_ns = 60;
  pending.confirmation_complete = true;
  EXPECT_EQ(monitor.update(pending).state, LocalizationState::kLocalized);
}

TEST(LocalizationMonitor, RejectsInvalidConfigurationAndNonMonotonicEvidence)
{
  auto invalid = config();
  invalid.lost_failures_required = invalid.degraded_failures_required - 1U;
  EXPECT_THROW(LocalizationMonitor{invalid}, std::invalid_argument);
  invalid = config();
  invalid.strong_maximum_fitness = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(LocalizationMonitor{invalid}, std::invalid_argument);

  LocalizationMonitor monitor{config()};
  monitor.update(evidence(10, true));
  EXPECT_THROW(monitor.update(evidence(10, true)), std::invalid_argument);
  EXPECT_THROW(monitor.beginRelocalization(9), std::invalid_argument);
}

TEST(LocalizationMonitor, GeneratedEventSequencesPreserveRecoveryInvariants)
{
  auto configured = config();
  configured.initialization_successes_required = 1U;
  configured.degraded_failures_required = 1U;
  configured.lost_failures_required = 2U;
  configured.relocalization_successes_required = 2U;
  for (std::uint32_t pattern = 0U; pattern < 256U; ++pattern) {
    LocalizationMonitor monitor{configured};
    std::int64_t stamp = 10;
    auto previous = monitor.update(evidence(stamp, true)).state;
    ASSERT_EQ(previous, LocalizationState::kLocalized);
    for (std::uint32_t index = 0U; index < 8U; ++index) {
      stamp += 10;
      const bool accepted = (pattern & (1U << index)) != 0U;
      const auto current = monitor.update(evidence(stamp, accepted)).state;
      if (previous == LocalizationState::kLost) {
        EXPECT_EQ(current, LocalizationState::kLost) << "pattern=" << pattern;
      }
      EXPECT_GE(static_cast<int>(current), static_cast<int>(LocalizationState::kInitializing));
      EXPECT_LE(static_cast<int>(current), static_cast<int>(LocalizationState::kRelocalizing));
      previous = current;
    }
  }
}

}  // namespace
}  // namespace a2w_fastlio_localization
