#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/nano_gicp_registration.hpp"
#include "a2w_fastlio_common/quatro_registration.hpp"

namespace a2w_fastlio_common
{
namespace
{

CloudPtr asymmetricCloud()
{
  auto cloud = CloudPtr{new Cloud{}};
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 6; ++y) {
      PointT point;
      point.x = 0.35F * static_cast<float>(x);
      point.y = 0.4F * static_cast<float>(y);
      point.z = 0.12F * static_cast<float>((x * x + 3 * y) % 7) +
        0.05F * static_cast<float>(x);
      point.intensity = static_cast<float>(x * 10 + y);
      cloud->push_back(point);
    }
  }
  return cloud;
}

Pose3d knownTransform()
{
  Pose3d pose;
  pose.rotation = Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ());
  pose.translation = Eigen::Vector3d{1.2, -0.7, 0.25};
  return pose;
}

CloudPtr transformed(const CloudConstPtr & source, const Pose3d & pose)
{
  auto target = CloudPtr{new Cloud{}};
  for (const auto & input : source->points) {
    const Eigen::Vector3d xyz =
      pose.rotation * input.getVector3fMap().cast<double>() + pose.translation;
    auto output = input;
    output.x = static_cast<float>(xyz.x());
    output.y = static_cast<float>(xyz.y());
    output.z = static_cast<float>(xyz.z());
    target->push_back(output);
  }
  return target;
}

double translationError(const Pose3d & actual, const Pose3d & expected)
{
  return (actual.translation - expected.translation).norm();
}

double rotationError(const Pose3d & actual, const Pose3d & expected)
{
  return Eigen::AngleAxisd(expected.rotation.inverse() * actual.rotation).angle();
}

TEST(RegistrationSynthetic, QuatroProducesUsefulCoarseTransform)
{
  const auto source = asymmetricCloud();
  const auto expected = knownTransform();
  const auto target = transformed(source, expected);
  const QuatroRegistration registration{QuatroRegistrationConfig{
      0.45, 0.8, 0.15, 1.4, 1e-4, 100, false, true, 30.0, 500, 20U}};

  const auto result = registration.align(source, target, std::nullopt);

  ASSERT_TRUE(isUsableRegistration(result)) << result.rejection_reason;
  EXPECT_LT(translationError(result.transform, expected), expected.translation.norm());
  EXPECT_LT(rotationError(result.transform, expected), 0.35);
}

TEST(RegistrationSynthetic, NanoGicpRefinesFromCoarseGuess)
{
  const auto source = asymmetricCloud();
  const auto expected = knownTransform();
  const auto target = transformed(source, expected);
  Pose3d coarse = expected;
  coarse.translation += Eigen::Vector3d{0.15, -0.1, 0.05};
  coarse.rotation = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitZ()) * coarse.rotation;
  const NanoGicpRegistration registration{NanoGicpRegistrationConfig{
      2.0, 1, 10, 64, 1e-6, 1e-6, 0, 0.1, 20U}};

  const auto result = registration.align(source, target, coarse);

  ASSERT_TRUE(isUsableRegistration(result)) << result.rejection_reason;
  EXPECT_LT(translationError(result.transform, expected), translationError(coarse, expected));
  EXPECT_LT(rotationError(result.transform, expected), rotationError(coarse, expected));
}

TEST(RegistrationSynthetic, AdaptersRejectEmptyAndNonFiniteClouds)
{
  const QuatroRegistration coarse{QuatroRegistrationConfig{}};
  const NanoGicpRegistration fine{NanoGicpRegistrationConfig{}};
  const auto empty = CloudPtr{new Cloud{}};
  auto invalid = asymmetricCloud();
  invalid->front().x = std::numeric_limits<float>::quiet_NaN();

  EXPECT_FALSE(coarse.align(empty, asymmetricCloud(), std::nullopt).success);
  EXPECT_FALSE(coarse.align(invalid, asymmetricCloud(), std::nullopt).success);
  EXPECT_FALSE(fine.align(empty, asymmetricCloud(), Pose3d{}).success);
  EXPECT_FALSE(fine.align(invalid, asymmetricCloud(), Pose3d{}).success);
}

TEST(RegistrationSynthetic, QuatroRejectsDegenerateLineGeometry)
{
  auto source = CloudPtr{new Cloud{}};
  for (int index = 0; index < 40; ++index) {
    PointT point;
    point.x = 0.1F * static_cast<float>(index);
    source->push_back(point);
  }
  const auto target = transformed(source, knownTransform());
  const QuatroRegistration registration{QuatroRegistrationConfig{}};

  const auto result = registration.align(source, target, std::nullopt);

  EXPECT_FALSE(result.success);
}

TEST(RegistrationSynthetic, AdaptersRejectInvalidConfiguration)
{
  auto coarse = QuatroRegistrationConfig{};
  coarse.fpfh_normal_radius_m = 0.0;
  EXPECT_THROW((QuatroRegistration{coarse}), std::invalid_argument);

  auto fine = NanoGicpRegistrationConfig{};
  fine.maximum_iterations = 0;
  EXPECT_THROW((NanoGicpRegistration{fine}), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_common
