#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/place_recognition.hpp"

namespace a2w_fastlio_common
{
namespace
{

TEST(PlaceRecognitionTypes, CandidateCarriesStableRankingEvidence)
{
  const LoopCandidate candidate{42U, 2U, 0.12, 0.5};

  EXPECT_EQ(candidate.keyframe_id, 42U);
  EXPECT_EQ(candidate.rank, 2U);
  EXPECT_DOUBLE_EQ(candidate.distance, 0.12);
  EXPECT_DOUBLE_EQ(candidate.yaw_hint_rad, 0.5);
}

TEST(PlaceRecognitionTypes, DescriptorOwnsValuesWithDeclaredShape)
{
  const ScanDescriptor descriptor{2U, 3U, {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F}};

  EXPECT_EQ(descriptor.rings(), 2U);
  EXPECT_EQ(descriptor.sectors(), 3U);
  EXPECT_EQ(descriptor.values().size(), 6U);
  EXPECT_FLOAT_EQ(descriptor.values().at(4), 5.0F);
}

TEST(PlaceRecognitionTypes, DescriptorRejectsZeroDimensions)
{
  EXPECT_THROW((ScanDescriptor{0U, 60U, {}}), std::invalid_argument);
  EXPECT_THROW((ScanDescriptor{20U, 0U, {}}), std::invalid_argument);
}

TEST(PlaceRecognitionTypes, DescriptorRejectsValueCountMismatch)
{
  EXPECT_THROW((ScanDescriptor{2U, 3U, {1.0F, 2.0F}}), std::invalid_argument);
}

TEST(PlaceRecognitionTypes, DescriptorRejectsNonFiniteValues)
{
  EXPECT_THROW(
    (ScanDescriptor{1U, 1U, {std::numeric_limits<float>::quiet_NaN()}}),
    std::invalid_argument);
}

TEST(PlaceRecognitionTypes, CandidateFilterDefaultsToWholeDatabase)
{
  const CandidateFilter filter{};

  EXPECT_EQ(filter.max_inclusive_id, std::numeric_limits<std::uint64_t>::max());
  EXPECT_EQ(filter.exclude_recent, 0U);
}

}  // namespace
}  // namespace a2w_fastlio_common
