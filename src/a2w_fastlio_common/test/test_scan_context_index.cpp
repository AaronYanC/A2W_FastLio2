#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/scan_context_index.hpp"

namespace a2w_fastlio_common
{
namespace
{

ScanDescriptor descriptor(std::initializer_list<float> values)
{
  return ScanDescriptor{2U, 4U, std::vector<float>{values}};
}

TEST(ScanContextIndex, ReturnsTopKOrderedByDistanceThenId)
{
  ScanContextIndex index;
  index.add(30U, descriptor({1, 0, 0, 0, 1, 0, 0, 0}));
  index.add(10U, descriptor({1, 0, 0, 0, 1, 0, 0, 0}));
  index.add(20U, descriptor({1, 0, 0, 0, 0.2F, 0, 0, 0}));

  const auto result = index.queryTopK(
    descriptor({1, 0, 0, 0, 1, 0, 0, 0}), 2U, CandidateFilter{});

  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].keyframe_id, 10U);
  EXPECT_EQ(result[0].rank, 0U);
  EXPECT_DOUBLE_EQ(result[0].distance, 0.0);
  EXPECT_EQ(result[1].keyframe_id, 30U);
  EXPECT_EQ(result[1].rank, 1U);
}

TEST(ScanContextIndex, ReportsQuarterTurnYawHint)
{
  ScanContextIndex index;
  index.add(7U, descriptor({0, 1, 0, 0, 0, 2, 0, 0}));

  const auto result = index.queryTopK(
    descriptor({1, 0, 0, 0, 2, 0, 0, 0}), 1U, CandidateFilter{});

  ASSERT_EQ(result.size(), 1U);
  EXPECT_NEAR(result.front().distance, 0.0, 1e-12);
  EXPECT_NEAR(std::abs(result.front().yaw_hint_rad), 1.5707963267948966, 1e-12);
}

TEST(ScanContextIndex, ExcludesConfiguredRecentIds)
{
  ScanContextIndex index;
  for (std::uint64_t id = 1U; id <= 4U; ++id) {
    index.add(id, descriptor({1, 0, 0, 0, 1, 0, 0, 0}));
  }

  const auto result = index.queryTopK(
    descriptor({1, 0, 0, 0, 1, 0, 0, 0}), 10U, CandidateFilter{4U, 2U});

  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].keyframe_id, 1U);
  EXPECT_EQ(result[1].keyframe_id, 2U);
}

TEST(ScanContextIndex, KeepsRepeatedStructuresAsDistinctCandidates)
{
  ScanContextIndex index;
  index.add(5U, descriptor({1, 0, 2, 0, 3, 0, 4, 0}));
  index.add(50U, descriptor({1, 0, 2, 0, 3, 0, 4, 0}));

  const auto result = index.queryTopK(
    descriptor({1, 0, 2, 0, 3, 0, 4, 0}), 5U, CandidateFilter{});

  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0].keyframe_id, 5U);
  EXPECT_EQ(result[1].keyframe_id, 50U);
  EXPECT_DOUBLE_EQ(result[0].distance, result[1].distance);
}

TEST(ScanContextIndex, RejectsDuplicateIdWithoutMutatingIndex)
{
  ScanContextIndex index;
  index.add(1U, descriptor({1, 0, 0, 0, 1, 0, 0, 0}));

  EXPECT_THROW(
    index.add(1U, descriptor({0, 1, 0, 0, 0, 1, 0, 0})),
    std::invalid_argument);
  EXPECT_EQ(index.size(), 1U);
}

TEST(ScanContextIndex, RejectsDimensionMismatchAndZeroK)
{
  ScanContextIndex index;
  index.add(1U, descriptor({1, 0, 0, 0, 1, 0, 0, 0}));

  EXPECT_THROW(
    index.queryTopK(ScanDescriptor{1U, 4U, {1, 0, 0, 0}}, 1U, CandidateFilter{}),
    std::invalid_argument);
  EXPECT_THROW(
    index.queryTopK(descriptor({1, 0, 0, 0, 1, 0, 0, 0}), 0U, CandidateFilter{}),
    std::invalid_argument);
  EXPECT_EQ(index.size(), 1U);
}

TEST(ScanContextIndex, EmptyIndexReturnsNoCandidates)
{
  const ScanContextIndex index;
  EXPECT_TRUE(index.queryTopK(
      descriptor({1, 0, 0, 0, 1, 0, 0, 0}), 3U, CandidateFilter{}).empty());
}

}  // namespace
}  // namespace a2w_fastlio_common
