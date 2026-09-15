#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_common/local_map_builder.hpp"

namespace a2w_fastlio_common
{
namespace
{

PointT point(float x, float y, float z)
{
  PointT result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = x + y + z;
  return result;
}

KeyFrame keyframe(std::uint64_t id, double translation_x, std::vector<PointT> points)
{
  KeyFrame result;
  result.id = id;
  result.optimized_pose.translation.x() = translation_x;
  result.body_cloud->assign(points.begin(), points.end());
  return result;
}

class MapKeyFrameProvider final : public KeyFrameProvider
{
public:
  explicit MapKeyFrameProvider(std::map<std::uint64_t, KeyFrame> frames)
  : frames_(std::move(frames)) {}

  std::optional<KeyFrame> get(std::uint64_t id) const override
  {
    const auto iterator = frames_.find(id);
    return iterator == frames_.end() ? std::nullopt : std::optional<KeyFrame>{iterator->second};
  }

  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    if (frames_.empty()) {
      return std::nullopt;
    }
    return std::pair<std::uint64_t, std::uint64_t>{frames_.begin()->first, frames_.rbegin()->first};
  }

private:
  std::map<std::uint64_t, KeyFrame> frames_;
};

TEST(LocalMapBuilder, ClampsNeighborhoodAndTransformsBodyClouds)
{
  const MapKeyFrameProvider provider{{
      {0U, keyframe(0U, 0.0, {point(0, 0, 0)})},
      {1U, keyframe(1U, 10.0, {point(1, 0, 0)})},
      {2U, keyframe(2U, 20.0, {point(2, 0, 0)})},
    }};
  const LocalMapBuilder builder{LocalMapConfig{0.0, 100U}};

  const auto result = builder.build(1U, 5U, 5U, provider);

  EXPECT_EQ(result.included_ids, (std::vector<std::uint64_t>{0U, 1U, 2U}));
  ASSERT_EQ(result.cloud->size(), 3U);
  EXPECT_FLOAT_EQ(result.cloud->at(0).x, 0.0F);
  EXPECT_FLOAT_EQ(result.cloud->at(1).x, 11.0F);
  EXPECT_FLOAT_EQ(result.cloud->at(2).x, 22.0F);
}

TEST(LocalMapBuilder, AppliesOptimizedRotation)
{
  auto frame = keyframe(0U, 0.0, {point(1, 0, 0)});
  frame.optimized_pose.rotation =
    Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ());
  const MapKeyFrameProvider provider{{{0U, frame}}};
  const LocalMapBuilder builder{LocalMapConfig{0.0, 100U}};

  const auto result = builder.build(0U, 0U, 0U, provider);

  ASSERT_EQ(result.cloud->size(), 1U);
  EXPECT_NEAR(result.cloud->front().x, 0.0F, 1e-6F);
  EXPECT_NEAR(result.cloud->front().y, 1.0F, 1e-6F);
}

TEST(LocalMapBuilder, RejectsMissingFrameInsideDeclaredBounds)
{
  const MapKeyFrameProvider provider{{
      {0U, keyframe(0U, 0.0, {point(0, 0, 0)})},
      {2U, keyframe(2U, 0.0, {point(0, 0, 0)})},
    }};
  const LocalMapBuilder builder{LocalMapConfig{0.0, 100U}};

  EXPECT_THROW(builder.build(0U, 0U, 2U, provider), std::runtime_error);
  EXPECT_THROW(builder.build(1U, 0U, 0U, provider), std::runtime_error);
}

TEST(LocalMapBuilder, RejectsEmptyProviderAndEmptyCloud)
{
  const LocalMapBuilder builder{LocalMapConfig{0.0, 100U}};
  EXPECT_THROW(builder.build(0U, 0U, 0U, MapKeyFrameProvider{{}}), std::runtime_error);

  const MapKeyFrameProvider provider{{{0U, keyframe(0U, 0.0, {})}}};
  EXPECT_THROW(builder.build(0U, 0U, 0U, provider), std::runtime_error);
}

TEST(LocalMapBuilder, VoxelFiltersAndCapsPointsDeterministically)
{
  const MapKeyFrameProvider provider{{
      {0U, keyframe(0U, 0.0, {
          point(0.01F, 0.01F, 0.01F), point(0.02F, 0.02F, 0.02F),
          point(1.01F, 0, 0), point(2.01F, 0, 0), point(3.01F, 0, 0)})},
    }};
  const LocalMapBuilder builder{LocalMapConfig{0.5, 2U}};

  const auto first = builder.build(0U, 0U, 0U, provider);
  const auto second = builder.build(0U, 0U, 0U, provider);

  ASSERT_EQ(first.cloud->size(), 2U);
  ASSERT_EQ(second.cloud->size(), 2U);
  EXPECT_FLOAT_EQ(
    (first.cloud->at(0).getVector3fMap() - second.cloud->at(0).getVector3fMap()).norm(), 0.0F);
  EXPECT_FLOAT_EQ(
    (first.cloud->at(1).getVector3fMap() - second.cloud->at(1).getVector3fMap()).norm(), 0.0F);
}

TEST(LocalMapBuilder, RejectsInvalidConfiguration)
{
  EXPECT_THROW((LocalMapBuilder{LocalMapConfig{-0.1, 100U}}), std::invalid_argument);
  EXPECT_THROW((LocalMapBuilder{LocalMapConfig{0.1, 0U}}), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_common
