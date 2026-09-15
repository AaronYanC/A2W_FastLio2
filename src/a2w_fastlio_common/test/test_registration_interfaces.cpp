#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_common
{
namespace
{

class FixedCoarseRegistration final : public CoarseRegistration
{
public:
  explicit FixedCoarseRegistration(RegistrationResult result)
  : result_(std::move(result)) {}

  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    return result_;
  }

private:
  RegistrationResult result_;
};

class FixedFineRegistration final : public FineRegistration
{
public:
  explicit FixedFineRegistration(RegistrationResult result)
  : result_(std::move(result)) {}

  RegistrationResult align(
    const CloudConstPtr &, const CloudConstPtr &,
    const std::optional<Pose3d> &) const override
  {
    return result_;
  }

private:
  RegistrationResult result_;
};

RegistrationResult acceptedResult()
{
  RegistrationResult result;
  result.success = true;
  result.converged = true;
  result.transform.translation = Eigen::Vector3d{1.0, 2.0, 3.0};
  result.fitness = 0.05;
  result.overlap = 0.8;
  result.correspondence_count = 100U;
  result.elapsed_ms = 4.0;
  return result;
}

TEST(RegistrationInterfaces, DefaultResultIsNotUsable)
{
  const RegistrationResult result;
  EXPECT_FALSE(isUsableRegistration(result));
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.converged);
}

TEST(RegistrationInterfaces, AcceptsFiniteConvergedResult)
{
  EXPECT_TRUE(isUsableRegistration(acceptedResult()));
}

TEST(RegistrationInterfaces, RejectsSuccessfulFlagWithNonFiniteTransform)
{
  auto result = acceptedResult();
  result.transform.translation.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(isUsableRegistration(result));
}

TEST(RegistrationInterfaces, RejectsInvalidMetrics)
{
  auto result = acceptedResult();
  result.overlap = 1.1;
  EXPECT_FALSE(isUsableRegistration(result));
  result = acceptedResult();
  result.fitness = -0.1;
  EXPECT_FALSE(isUsableRegistration(result));
  result = acceptedResult();
  result.elapsed_ms = -1.0;
  EXPECT_FALSE(isUsableRegistration(result));
}

TEST(RegistrationInterfaces, PreservesFailureReason)
{
  RegistrationResult result;
  result.rejection_reason = "empty_source";
  EXPECT_EQ(result.rejection_reason, "empty_source");
  EXPECT_FALSE(isUsableRegistration(result));
}

TEST(RegistrationInterfaces, ConcreteAlgorithmsAreCallableThroughAbstractInterfaces)
{
  const auto expected = acceptedResult();
  const std::unique_ptr<CoarseRegistration> coarse =
    std::make_unique<FixedCoarseRegistration>(expected);
  const std::unique_ptr<FineRegistration> fine =
    std::make_unique<FixedFineRegistration>(expected);
  const auto cloud = CloudPtr{new Cloud{}};

  EXPECT_TRUE(isUsableRegistration(coarse->align(cloud, cloud, std::nullopt)));
  EXPECT_TRUE(isUsableRegistration(fine->align(cloud, cloud, expected.transform)));
}

}  // namespace
}  // namespace a2w_fastlio_common
