#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/match_validator.hpp"

namespace a2w_fastlio_common
{
namespace
{

RegistrationResult goodResult()
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.fitness = 0.05;
  result.overlap = 0.8;
  result.correspondence_count = 80U;
  result.elapsed_ms = 2.0;
  return result;
}

MatchValidator validator()
{
  return MatchValidator{MatchValidationConfig{0.2, 0.5, 30U, 2.0, 0.5, 0.05}};
}

TEST(MatchValidator, AcceptsResultWithGeometricEvidence)
{
  const auto validation = validator().validate(goodResult(), ValidationContext{});
  EXPECT_TRUE(validation.accepted) << validation.reason;
}

TEST(MatchValidator, RejectsEveryMetricWithExplicitReason)
{
  auto result = goodResult();
  result.converged = false;
  EXPECT_EQ(validator().validate(result, {}).reason, "registration_not_usable");
  result = goodResult();
  result.fitness = 0.3;
  EXPECT_EQ(validator().validate(result, {}).reason, "fitness_above_limit");
  result = goodResult();
  result.overlap = 0.4;
  EXPECT_EQ(validator().validate(result, {}).reason, "overlap_below_limit");
  result = goodResult();
  result.correspondence_count = 20U;
  EXPECT_EQ(validator().validate(result, {}).reason, "correspondences_below_limit");
}

TEST(MatchValidator, RejectsPoseJumpRelativeToPrediction)
{
  ValidationContext context;
  context.predicted_transform = Pose3d{};
  auto result = goodResult();
  result.transform.translation.x() = 2.1;
  EXPECT_EQ(validator().validate(result, context).reason, "translation_jump_above_limit");

  result = goodResult();
  result.transform.rotation = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ());
  EXPECT_EQ(validator().validate(result, context).reason, "rotation_jump_above_limit");
}

TEST(MatchValidator, RejectsAmbiguousRepeatedStructureCandidates)
{
  ValidationContext context;
  context.best_descriptor_distance = 0.10;
  context.second_descriptor_distance = 0.13;
  EXPECT_EQ(validator().validate(goodResult(), context).reason, "candidate_separation_below_limit");

  context.second_descriptor_distance = 0.18;
  EXPECT_TRUE(validator().validate(goodResult(), context).accepted);
}

TEST(MatchValidator, RejectsInvalidConfiguration)
{
  auto config = MatchValidationConfig{};
  config.minimum_overlap = 1.1;
  EXPECT_THROW((MatchValidator{config}), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_common
