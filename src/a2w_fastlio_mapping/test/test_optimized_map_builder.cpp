#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <utility>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/optimized_map_builder.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

using namespace a2w_fastlio_common;

PointT point(float x, float y, float z = 0.0F)
{
  PointT result;
  result.x = x;
  result.y = y;
  result.z = z;
  result.intensity = x + y + z;
  return result;
}

KeyFrame frame(std::uint64_t id, std::initializer_list<PointT> points)
{
  KeyFrame result;
  result.id = id;
  result.body_cloud->assign(points.begin(), points.end());
  return result;
}

class Provider final : public KeyFrameProvider
{
public:
  explicit Provider(std::map<std::uint64_t, KeyFrame> frames) : frames_{std::move(frames)} {}
  std::optional<KeyFrame> get(std::uint64_t id) const override
  {
    const auto iterator = frames_.find(id);
    return iterator == frames_.end() ? std::nullopt : std::optional<KeyFrame>{iterator->second};
  }
  std::optional<std::pair<std::uint64_t, std::uint64_t>> bounds() const override
  {
    if (frames_.empty()) {return std::nullopt;}
    return std::pair<std::uint64_t, std::uint64_t>{frames_.begin()->first, frames_.rbegin()->first};
  }
private:
  std::map<std::uint64_t, KeyFrame> frames_;
};

OptimizedPoseSnapshot snapshot(std::initializer_list<OptimizedPose> poses)
{
  OptimizedPoseSnapshot result;
  result.revision = 1U;
  result.poses.assign(poses.begin(), poses.end());
  return result;
}

TEST(OptimizedMapBuilder, FullMapUsesSnapshotPosesWithoutMutatingKeyframes)
{
  const Provider provider{{
      {0U, frame(0U, {point(1.0F, 0.0F)})},
      {1U, frame(1U, {point(0.0F, 1.0F)})},
    }};
  Pose3d pose1;
  pose1.translation.x() = 10.0;
  pose1.rotation = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ());
  const auto poses = snapshot({OptimizedPose{0U, Pose3d{}}, OptimizedPose{1U, pose1}});
  const OptimizedMapBuilder builder{OptimizedMapConfig{0.0, 0.5, 100U}};

  const auto full = builder.buildFull(provider, poses);

  ASSERT_EQ(full->size(), 2U);
  EXPECT_FLOAT_EQ(full->at(0).x, 1.0F);
  EXPECT_NEAR(full->at(1).x, 9.0F, 1e-6F);
  EXPECT_NEAR(full->at(1).y, 0.0F, 1e-6F);
  ASSERT_EQ(provider.get(1U)->body_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(provider.get(1U)->body_cloud->front().x, 0.0F);
}

TEST(OptimizedMapBuilder, PreviewIsSeparateVoxelFilteredAndDeterministicallyBounded)
{
  KeyFrame dense;
  dense.id = 0U;
  for (int index = 0; index < 1000; ++index) {
    dense.body_cloud->push_back(point(
      0.01F * static_cast<float>(index),
      0.01F * static_cast<float>(index % 7)));
  }
  const Provider provider{{{0U, dense}}};
  const auto poses = snapshot({OptimizedPose{0U, Pose3d{}}});
  const OptimizedMapBuilder builder{OptimizedMapConfig{0.0, 0.25, 20U}};

  const auto full = builder.buildFull(provider, poses);
  const auto first_preview = builder.buildPreview(provider, poses);
  const auto second_preview = builder.buildPreview(provider, poses);

  EXPECT_EQ(full->size(), 1000U);
  ASSERT_LE(first_preview->size(), 20U);
  ASSERT_EQ(first_preview->size(), second_preview->size());
  for (std::size_t index = 0; index < first_preview->size(); ++index) {
    EXPECT_EQ(first_preview->at(index).getVector4fMap(), second_preview->at(index).getVector4fMap());
  }
}

TEST(OptimizedMapBuilder, FullMapMayUseIndependentVoxelSetting)
{
  const Provider provider{{
      {0U, frame(0U, {point(0.01F, 0.01F), point(0.02F, 0.02F), point(1.0F, 0.0F)})},
    }};
  const auto poses = snapshot({OptimizedPose{0U, Pose3d{}}});
  const OptimizedMapBuilder builder{OptimizedMapConfig{0.5, 1.0, 100U}};
  EXPECT_EQ(builder.buildFull(provider, poses)->size(), 2U);
}

TEST(OptimizedMapBuilder, RejectsMissingNonContiguousAndNonFiniteInput)
{
  const Provider provider{{{0U, frame(0U, {point(0.0F, 0.0F)})}}};
  OptimizedMapBuilder builder{OptimizedMapConfig{0.0, 0.5, 100U}};
  EXPECT_THROW(builder.buildFull(provider, OptimizedPoseSnapshot{}), std::invalid_argument);
  auto bad_id = snapshot({OptimizedPose{1U, Pose3d{}}});
  EXPECT_THROW(builder.buildFull(provider, bad_id), std::invalid_argument);
  auto bad_pose = snapshot({OptimizedPose{0U, Pose3d{}}});
  bad_pose.poses.front().pose.translation.x() = std::numeric_limits<double>::infinity();
  EXPECT_THROW(builder.buildPreview(provider, bad_pose), std::invalid_argument);
}

TEST(OptimizedMapBuilder, RejectsInvalidConfiguration)
{
  EXPECT_THROW((OptimizedMapBuilder{OptimizedMapConfig{-0.1, 0.5, 10U}}), std::invalid_argument);
  EXPECT_THROW((OptimizedMapBuilder{OptimizedMapConfig{0.0, 0.0, 10U}}), std::invalid_argument);
  EXPECT_THROW((OptimizedMapBuilder{OptimizedMapConfig{0.0, 0.5, 0U}}), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
