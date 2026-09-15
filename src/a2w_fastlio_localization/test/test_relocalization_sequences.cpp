#include <gtest/gtest.h>

#include "a2w_fastlio_localization/global_relocalizer.hpp"

namespace a2w_fastlio_localization
{
namespace
{

RelocalizationResult match(const std::uint64_t id, const double x, const bool strong = false)
{
  RelocalizationResult result;
  result.success = true;
  result.strong = strong;
  result.candidate_id = id;
  result.map_body.translation.x() = x;
  return result;
}

RelocalizationSessionConfig config()
{
  return RelocalizationSessionConfig{3U, 0.5, 0.2, 500};
}

TEST(RelocalizationSession, RequiresConsistentMultiFrameEvidence)
{
  RelocalizationSession session{config()};
  session.start(10);
  EXPECT_FALSE(session.observe(20, match(7U, 10.0)).confirmed_correction.has_value());
  EXPECT_FALSE(session.observe(30, match(8U, 10.0)).confirmed_correction.has_value());
  EXPECT_EQ(session.latest().consecutive_matches, 1U);
  EXPECT_FALSE(session.observe(40, match(8U, 10.2)).confirmed_correction.has_value());
  const auto confirmed = session.observe(50, match(8U, 10.1));
  ASSERT_TRUE(confirmed.confirmed_correction.has_value());
  EXPECT_EQ(confirmed.candidate_id, 8U);
  EXPECT_DOUBLE_EQ(confirmed.confirmed_correction->translation.x(), 10.1);
}

TEST(RelocalizationSession, AmbiguityAndTransformJumpResetConfirmation)
{
  RelocalizationSession session{config()};
  session.start(10);
  session.observe(20, match(7U, 10.0));
  auto ambiguous = RelocalizationResult{};
  ambiguous.reason = "candidate_score_ambiguous";
  EXPECT_EQ(session.observe(30, ambiguous).consecutive_matches, 0U);
  session.observe(40, match(7U, 10.0));
  EXPECT_EQ(session.observe(50, match(7U, 12.0)).consecutive_matches, 1U);
}

TEST(RelocalizationSession, StrongResultConfirmsOnceAndTimeoutOrCancelStopsSession)
{
  RelocalizationSession session{config()};
  session.start(10);
  EXPECT_TRUE(session.observe(20, match(7U, 3.0, true)).confirmed_correction.has_value());
  EXPECT_FALSE(session.latest().active);
  EXPECT_THROW(session.observe(30, match(7U, 3.0)), std::logic_error);

  session.start(40);
  const auto timeout = session.observe(540, match(7U, 3.0));
  EXPECT_FALSE(timeout.active);
  EXPECT_EQ(timeout.reason, "session_timeout");

  session.start(600);
  EXPECT_EQ(session.cancel(610).reason, "cancelled");
  EXPECT_FALSE(session.latest().active);
}

TEST(RelocalizationSession, RejectsInvalidConfigAndNonMonotonicTime)
{
  auto invalid = config();
  invalid.confirmation_count = 1U;
  EXPECT_THROW(RelocalizationSession{invalid}, std::invalid_argument);
  RelocalizationSession session{config()};
  session.start(10);
  EXPECT_THROW(session.observe(10, match(1U, 0.0)), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_localization
