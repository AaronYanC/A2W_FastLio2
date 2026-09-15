#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/scan_context_place_recognition.hpp"

namespace a2w_fastlio_common
{
namespace
{

CloudPtr makeCloud(std::initializer_list<PointT> points)
{
  auto cloud = CloudPtr{new Cloud{}};
  cloud->assign(points.begin(), points.end());
  return cloud;
}

PointT makePoint(float x, float y, float z)
{
  PointT point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = 1.0F;
  return point;
}

TEST(ScanContextDescriptor, EncodesMaximumHeightInPolarBins)
{
  const ScanContextConfig config{2U, 4U, 10.0, 0.0};
  const ScanContextPlaceRecognition recognition{config};
  const auto cloud = makeCloud({
      makePoint(1.0F, 0.0F, 2.0F),
      makePoint(2.0F, 0.0F, 1.0F),
      makePoint(-0.173648F, 0.984808F, 4.0F),
      makePoint(6.0F, 0.0F, 3.0F),
    });

  const auto descriptor = recognition.describe(cloud);

  ASSERT_EQ(descriptor.values().size(), 8U);
  const std::vector<float> expected{2.0F, 4.0F, 0.0F, 0.0F, 3.0F, 0.0F, 0.0F, 0.0F};
  EXPECT_EQ(descriptor.values(), expected);
}

TEST(ScanContextDescriptor, IgnoresPointsOutsideConfiguredRadius)
{
  const ScanContextPlaceRecognition recognition{ScanContextConfig{1U, 4U, 5.0, 0.0}};
  const auto descriptor = recognition.describe(makeCloud({
      makePoint(1.0F, 0.0F, 2.0F),
      makePoint(6.0F, 0.0F, 9.0F),
    }));

  EXPECT_EQ(descriptor.values(), (std::vector<float>{2.0F, 0.0F, 0.0F, 0.0F}));
}

TEST(ScanContextDescriptor, AddsConfiguredSensorHeight)
{
  const ScanContextPlaceRecognition recognition{ScanContextConfig{1U, 1U, 5.0, 2.0}};
  const auto descriptor = recognition.describe(makeCloud({makePoint(1.0F, 0.0F, -1.0F)}));

  ASSERT_EQ(descriptor.values().size(), 1U);
  EXPECT_FLOAT_EQ(descriptor.values().front(), 1.0F);
}

TEST(ScanContextDescriptor, RejectsEmptyOrNonFiniteCloud)
{
  const ScanContextPlaceRecognition recognition{ScanContextConfig{20U, 60U, 80.0, 2.0}};
  EXPECT_THROW(recognition.describe(CloudPtr{new Cloud{}}), std::invalid_argument);
  EXPECT_THROW(
    recognition.describe(makeCloud({
      makePoint(std::numeric_limits<float>::quiet_NaN(), 0.0F, 1.0F)})),
    std::invalid_argument);
}

TEST(ScanContextDescriptor, RejectsInvalidConfiguration)
{
  EXPECT_THROW((ScanContextPlaceRecognition{ScanContextConfig{0U, 60U, 80.0, 2.0}}),
    std::invalid_argument);
  EXPECT_THROW((ScanContextPlaceRecognition{ScanContextConfig{20U, 0U, 80.0, 2.0}}),
    std::invalid_argument);
  EXPECT_THROW((ScanContextPlaceRecognition{ScanContextConfig{20U, 60U, 0.0, 2.0}}),
    std::invalid_argument);
  EXPECT_THROW((ScanContextPlaceRecognition{ScanContextConfig{20U, 60U, 80.0,
      std::numeric_limits<double>::infinity()}}), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_common
