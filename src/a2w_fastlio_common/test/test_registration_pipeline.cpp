#include <memory>
#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/registration_pipeline.hpp"

namespace a2w_fastlio_common
{
namespace
{

CloudPtr cloud()
{
  auto result = CloudPtr{new Cloud{}};
  for (int index = 0; index < 40; ++index) {
    PointT point;
    point.x = 0.1F * static_cast<float>(index);
    point.y = 0.2F * static_cast<float>(index % 5);
    point.z = 0.1F * static_cast<float>(index % 3);
    result->push_back(point);
  }
  return result;
}

RegistrationResult resultAt(double x)
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.transform.translation.x() = x;
  result.fitness = 0.0;
  result.elapsed_ms = 1.0;
  return result;
}

class RecordingCoarse final : public CoarseRegistration
{
public:
  explicit RecordingCoarse(RegistrationResult result) : result_{std::move(result)} {}
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    ++calls;
    return result_;
  }
  mutable int calls{0};
private:
  RegistrationResult result_;
};

class RecordingFine final : public FineRegistration
{
public:
  explicit RecordingFine(RegistrationResult result) : result_{std::move(result)} {}
  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> & guess) const override
  {
    ++calls;
    received_guess = guess;
    return result_;
  }
  mutable int calls{0};
  mutable std::optional<Pose3d> received_guess{};
private:
  RegistrationResult result_;
};

RegistrationPipeline pipeline(
  std::shared_ptr<const CoarseRegistration> coarse,
  std::shared_ptr<const FineRegistration> fine)
{
  return RegistrationPipeline{
    std::move(coarse), std::move(fine),
    MatchValidator{MatchValidationConfig{0.05, 0.8, 30U, 5.0, 1.0, 0.05}},
    RegistrationPipelineConfig{0.25}};
}

TEST(RegistrationPipeline, PassesCoarseTransformAsFineInitialGuess)
{
  const auto coarse = std::make_shared<RecordingCoarse>(resultAt(0.0));
  const auto fine = std::make_shared<RecordingFine>(resultAt(0.0));
  const auto points = cloud();

  const auto output = pipeline(coarse, fine).run(points, points, std::nullopt);

  ASSERT_TRUE(output.success) << output.validation.reason;
  ASSERT_TRUE(fine->received_guess.has_value());
  EXPECT_DOUBLE_EQ(fine->received_guess->translation.x(), 0.0);
  EXPECT_EQ(output.fine.correspondence_count, points->size());
  EXPECT_DOUBLE_EQ(output.final_transform.translation.x(), 0.0);
}

TEST(RegistrationPipeline, CoarseFailureShortCircuitsFineRegistration)
{
  auto failed = RegistrationResult{};
  failed.rejection_reason = "no_correspondences";
  const auto coarse = std::make_shared<RecordingCoarse>(failed);
  const auto fine = std::make_shared<RecordingFine>(resultAt(0.0));
  const auto points = cloud();

  const auto output = pipeline(coarse, fine).run(points, points, std::nullopt);

  EXPECT_FALSE(output.success);
  EXPECT_EQ(output.validation.reason, "coarse_registration_failed:no_correspondences");
  EXPECT_EQ(fine->calls, 0);
}

TEST(RegistrationPipeline, SharedEvidenceRejectsFalseConvergedCandidate)
{
  const auto coarse = std::make_shared<RecordingCoarse>(resultAt(0.0));
  const auto fine = std::make_shared<RecordingFine>(resultAt(20.0));
  const auto points = cloud();

  const auto output = pipeline(coarse, fine).run(points, points, std::nullopt);

  EXPECT_FALSE(output.success);
  EXPECT_EQ(output.fine.correspondence_count, 0U);
  EXPECT_EQ(output.validation.reason, "fitness_above_limit");
}

TEST(RegistrationPipeline, CandidateAmbiguityIsValidatedAfterFineAlignment)
{
  const auto coarse = std::make_shared<RecordingCoarse>(resultAt(0.0));
  const auto fine = std::make_shared<RecordingFine>(resultAt(0.0));
  ValidationContext context;
  context.best_descriptor_distance = 0.1;
  context.second_descriptor_distance = 0.12;
  const auto points = cloud();

  const auto output = pipeline(coarse, fine).run(points, points, std::nullopt, context);

  EXPECT_FALSE(output.success);
  EXPECT_EQ(output.validation.reason, "candidate_separation_below_limit");
}

TEST(RegistrationPipeline, RejectsNullAlgorithmsAndInvalidEvidenceDistance)
{
  const auto coarse = std::make_shared<RecordingCoarse>(resultAt(0.0));
  const auto fine = std::make_shared<RecordingFine>(resultAt(0.0));
  const MatchValidator validator{MatchValidationConfig{}};
  EXPECT_THROW(
    (RegistrationPipeline{nullptr, fine, validator, RegistrationPipelineConfig{0.2}}),
    std::invalid_argument);
  EXPECT_THROW(
    (RegistrationPipeline{coarse, nullptr, validator, RegistrationPipelineConfig{0.2}}),
    std::invalid_argument);
  EXPECT_THROW(
    (RegistrationPipeline{coarse, fine, validator, RegistrationPipelineConfig{0.0}}),
    std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_common
