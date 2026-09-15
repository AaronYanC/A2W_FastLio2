#include <gtest/gtest.h>

#include "a2w_fastlio_localization/localization_manager.hpp"

namespace a2w_fastlio_localization
{
namespace
{

a2w_fastlio_common::FrontendFrame frame(const std::int64_t stamp, const double x)
{
  a2w_fastlio_common::FrontendFrame result;
  result.stamp_ns = stamp;
  result.odom_pose.translation.x() = x;
  auto cloud = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  cloud->push_back(a2w_fastlio_common::PointT{});
  result.body_cloud = cloud;
  return result;
}

MapMatchResult accepted(const double map_x)
{
  MapMatchResult result;
  result.success = true;
  result.reason = "accepted";
  result.map_body.translation.x() = map_x;
  result.registration.success = true;
  return result;
}

TEST(LocalizationManager, InitialMatchCreatesCorrectionAndHighRatePoseContinues)
{
  int calls = 0;
  LocalizationManager manager{{100, 5.0, 1.0}, [&](const auto &, const auto &) {
      ++calls;
      return accepted(10.0);
    }};
  const auto first = manager.process(frame(1000, 2.0));
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.match_accepted);
  EXPECT_DOUBLE_EQ(first.map_camera_init.translation.x(), 8.0);
  EXPECT_DOUBLE_EQ(first.map_body.translation.x(), 10.0);
  EXPECT_EQ(first.correction_age_ns, 0);

  const auto high_rate = manager.process(frame(1050, 3.0));
  EXPECT_TRUE(high_rate.valid);
  EXPECT_FALSE(high_rate.match_attempted);
  EXPECT_DOUBLE_EQ(high_rate.map_body.translation.x(), 11.0);
  EXPECT_EQ(high_rate.correction_age_ns, 50);
  EXPECT_EQ(calls, 1);
}

TEST(LocalizationManager, FailedMatchDoesNotRefreshCorrectionAge)
{
  int calls = 0;
  LocalizationManager manager{{100, 5.0, 1.0}, [&](const auto &, const auto &) {
      return calls++ == 0 ? accepted(0.0) : MapMatchResult{};
    }};
  ASSERT_TRUE(manager.process(frame(1000, 0.0)).valid);
  const auto failed = manager.process(frame(1100, 1.0));
  EXPECT_TRUE(failed.valid);
  EXPECT_TRUE(failed.match_attempted);
  EXPECT_FALSE(failed.match_accepted);
  EXPECT_EQ(failed.correction_stamp_ns, 1000);
  EXPECT_EQ(failed.correction_age_ns, 100);
}

TEST(LocalizationManager, RejectsJumpAndNonMonotonicFrames)
{
  int calls = 0;
  LocalizationManager manager{{1, 2.0, 0.5}, [&](const auto &, const auto &) {
      return calls++ == 0 ? accepted(0.0) : accepted(100.0);
    }};
  ASSERT_TRUE(manager.process(frame(10, 0.0)).valid);
  const auto jumped = manager.process(frame(11, 0.0));
  EXPECT_FALSE(jumped.match_accepted);
  EXPECT_EQ(jumped.reason, "correction_jump_rejected");
  EXPECT_DOUBLE_EQ(jumped.map_body.translation.x(), 0.0);
  EXPECT_THROW(manager.process(frame(11, 0.0)), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_localization
