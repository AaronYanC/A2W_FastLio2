#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/keyframe_manager.hpp"

namespace
{

using a2w_fastlio_common::Cloud;
using a2w_fastlio_common::FrontendFrame;
using a2w_fastlio_common::PointT;
using a2w_fastlio_mapping::KeyframeConfig;
using a2w_fastlio_mapping::KeyframeDecisionReason;
using a2w_fastlio_mapping::KeyframeManager;

FrontendFrame makeFrame(
  const std::int64_t stamp_ns, const double x, const double yaw_rad = 0.0)
{
  FrontendFrame frame;
  frame.stamp_ns = stamp_ns;
  frame.odom_pose.translation = Eigen::Vector3d{x, 0.0, 0.0};
  frame.odom_pose.rotation = Eigen::Quaterniond{
    Eigen::AngleAxisd{yaw_rad, Eigen::Vector3d::UnitZ()}};
  auto cloud = Cloud::Ptr{new Cloud{}};
  cloud->push_back(PointT{1.0F, 2.0F, 3.0F, 4.0F});
  frame.body_cloud = cloud;
  return frame;
}

KeyframeConfig testConfig()
{
  return KeyframeConfig{1.0, 10.0 * M_PI / 180.0, 2.0};
}

TEST(KeyframeManager, AcceptsFirstValidFrameAsKeyframeZero)
{
  KeyframeManager manager{testConfig()};

  const auto decision = manager.consider(makeFrame(1'000'000'000LL, 0.0));

  ASSERT_TRUE(decision.keyframe.has_value());
  EXPECT_EQ(decision.reason, KeyframeDecisionReason::kAcceptedFirst);
  EXPECT_EQ(decision.keyframe->id, 0U);
  EXPECT_EQ(decision.keyframe->stamp_ns, 1'000'000'000LL);
  EXPECT_TRUE(decision.keyframe->optimized_pose.translation.isZero());
  EXPECT_EQ(manager.size(), 1U);
}

TEST(KeyframeManager, RejectsFrameBelowEveryThresholdWithoutAdvancingState)
{
  KeyframeManager manager{testConfig()};
  ASSERT_TRUE(manager.consider(makeFrame(1'000'000'000LL, 0.0)).keyframe.has_value());

  const auto decision = manager.consider(makeFrame(1'500'000'000LL, 0.5));

  EXPECT_FALSE(decision.keyframe.has_value());
  EXPECT_EQ(decision.reason, KeyframeDecisionReason::kRejectedBelowThreshold);
  EXPECT_EQ(manager.size(), 1U);
}

TEST(KeyframeManager, AcceptsWhenAnyThresholdIsReached)
{
  struct TestCase
  {
    const char * name;
    std::int64_t stamp_ns;
    double x;
    double yaw_rad;
    KeyframeDecisionReason expected_reason;
  };

  const TestCase cases[] = {
    {"translation", 1'500'000'000LL, 1.0, 0.0,
      KeyframeDecisionReason::kAcceptedTranslation},
    {"rotation", 1'500'000'000LL, 0.0, 10.0 * M_PI / 180.0,
      KeyframeDecisionReason::kAcceptedRotation},
    {"elapsed time", 3'000'000'000LL, 0.0, 0.0,
      KeyframeDecisionReason::kAcceptedMaxInterval},
  };

  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.name);
    KeyframeManager manager{testConfig()};
    ASSERT_TRUE(manager.consider(makeFrame(1'000'000'000LL, 0.0)).keyframe.has_value());

    const auto decision = manager.consider(
      makeFrame(test_case.stamp_ns, test_case.x, test_case.yaw_rad));

    ASSERT_TRUE(decision.keyframe.has_value());
    EXPECT_EQ(decision.reason, test_case.expected_reason);
    EXPECT_EQ(decision.keyframe->id, 1U);
    EXPECT_EQ(manager.size(), 2U);
  }
}

TEST(KeyframeManager, UsesLastAcceptedKeyframeAndKeepsAcceptedIdsContiguous)
{
  KeyframeManager manager{testConfig()};
  ASSERT_EQ(manager.consider(makeFrame(1'000'000'000LL, 0.0)).keyframe->id, 0U);
  ASSERT_FALSE(manager.consider(makeFrame(1'500'000'000LL, 0.5)).keyframe.has_value());
  ASSERT_EQ(manager.consider(makeFrame(2'000'000'000LL, 1.0)).keyframe->id, 1U);
  ASSERT_FALSE(manager.consider(makeFrame(2'500'000'000LL, 1.5)).keyframe.has_value());

  const auto third = manager.consider(makeFrame(3'000'000'000LL, 2.0));

  ASSERT_TRUE(third.keyframe.has_value());
  EXPECT_EQ(third.keyframe->id, 2U);
  EXPECT_EQ(manager.size(), 3U);
}

TEST(KeyframeManager, RejectsValuesJustBelowAllThresholds)
{
  KeyframeManager manager{testConfig()};
  ASSERT_TRUE(manager.consider(makeFrame(1'000'000'000LL, 0.0)).keyframe.has_value());

  const auto decision = manager.consider(
    makeFrame(2'990'000'000LL, 0.99, 9.9 * M_PI / 180.0));

  EXPECT_FALSE(decision.keyframe.has_value());
  EXPECT_EQ(decision.reason, KeyframeDecisionReason::kRejectedBelowThreshold);
  EXPECT_EQ(manager.size(), 1U);
}

TEST(KeyframeManager, RejectsEmptyCloudWithoutMutatingState)
{
  KeyframeManager manager{testConfig()};
  auto frame = makeFrame(1'000'000'000LL, 0.0);
  frame.body_cloud = Cloud::ConstPtr{new Cloud{}};

  const auto rejected = manager.consider(frame);

  EXPECT_FALSE(rejected.keyframe.has_value());
  EXPECT_EQ(rejected.reason, KeyframeDecisionReason::kRejectedEmptyCloud);
  EXPECT_EQ(manager.size(), 0U);
  EXPECT_EQ(manager.consider(makeFrame(2'000'000'000LL, 0.0)).keyframe->id, 0U);
}

TEST(KeyframeManager, RejectsNonFiniteTranslationAndInvalidQuaternion)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto nan_translation = makeFrame(1'000'000'000LL, 0.0);
  nan_translation.odom_pose.translation.x() = nan;
  auto zero_quaternion = makeFrame(1'000'000'000LL, 0.0);
  zero_quaternion.odom_pose.rotation = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
  auto nan_quaternion = makeFrame(1'000'000'000LL, 0.0);
  nan_quaternion.odom_pose.rotation = Eigen::Quaterniond{nan, 0.0, 0.0, 0.0};

  for (const auto * frame : {&nan_translation, &zero_quaternion, &nan_quaternion}) {
    KeyframeManager manager{testConfig()};
    const auto decision = manager.consider(*frame);
    EXPECT_FALSE(decision.keyframe.has_value());
    EXPECT_EQ(decision.reason, KeyframeDecisionReason::kRejectedInvalidPose);
    EXPECT_EQ(manager.size(), 0U);
  }
}

TEST(KeyframeManager, RejectsTimestampThatIsNotNewerWithoutMutatingState)
{
  for (const auto stamp_ns : {1'000'000'000LL, 999'999'999LL}) {
    KeyframeManager manager{testConfig()};
    ASSERT_TRUE(manager.consider(makeFrame(1'000'000'000LL, 0.0)).keyframe.has_value());

    const auto decision = manager.consider(makeFrame(stamp_ns, 2.0));

    EXPECT_FALSE(decision.keyframe.has_value());
    EXPECT_EQ(decision.reason, KeyframeDecisionReason::kRejectedNonMonotonicTimestamp);
    EXPECT_EQ(manager.size(), 1U);
  }
}

TEST(KeyframeManager, RejectsNonPositiveOrNonFiniteConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const KeyframeConfig invalid_configs[] = {
    {0.0, 0.1, 1.0},
    {-1.0, 0.1, 1.0},
    {1.0, 0.0, 1.0},
    {1.0, -0.1, 1.0},
    {1.0, 0.1, 0.0},
    {1.0, 0.1, -1.0},
    {nan, 0.1, 1.0},
    {1.0, nan, 1.0},
    {1.0, 0.1, nan},
  };

  for (const auto & config : invalid_configs) {
    EXPECT_THROW(KeyframeManager{config}, std::invalid_argument);
  }
}

TEST(KeyframeManager, ResetClearsCountIdAndTimestampHistory)
{
  KeyframeManager manager{testConfig()};
  ASSERT_TRUE(manager.consider(makeFrame(10'000'000'000LL, 0.0)).keyframe.has_value());
  ASSERT_TRUE(manager.consider(makeFrame(11'000'000'000LL, 1.0)).keyframe.has_value());

  manager.reset();

  EXPECT_EQ(manager.size(), 0U);
  const auto restarted = manager.consider(makeFrame(1'000'000'000LL, 0.0));
  ASSERT_TRUE(restarted.keyframe.has_value());
  EXPECT_EQ(restarted.reason, KeyframeDecisionReason::kAcceptedFirst);
  EXPECT_EQ(restarted.keyframe->id, 0U);
}

}  // namespace
