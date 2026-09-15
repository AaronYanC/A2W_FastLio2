#include <gtest/gtest.h>

#include "a2w_fastlio_common/default_algorithm_suite.hpp"

namespace a2w_fastlio_common
{
namespace
{

TEST(DefaultAlgorithmSuite, ReturnsOnlyAbstractAlgorithmServices)
{
  const auto suite = createDefaultAlgorithmSuite(DefaultAlgorithmSuiteConfig{});

  ASSERT_NE(suite.place_recognition, nullptr);
  ASSERT_NE(suite.descriptor_index, nullptr);
  ASSERT_NE(suite.registration, nullptr);
  EXPECT_EQ(suite.descriptor_index->queryTopK(
    ScanDescriptor{1U, 1U, {0.0F}}, 1U, CandidateFilter{}).size(), 0U);
}

TEST(DefaultAlgorithmSuite, RejectsInvalidConfigurationThroughFactoryBoundary)
{
  auto config = DefaultAlgorithmSuiteConfig{};
  config.place_recognition.rings = 0U;
  EXPECT_THROW(createDefaultAlgorithmSuite(config), std::invalid_argument);

  config = DefaultAlgorithmSuiteConfig{};
  config.coarse.minimum_points = 0U;
  EXPECT_THROW(createDefaultAlgorithmSuite(config), std::invalid_argument);

  config = DefaultAlgorithmSuiteConfig{};
  config.fine.maximum_iterations = 0;
  EXPECT_THROW(createDefaultAlgorithmSuite(config), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_common
