#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_map/descriptor_database.hpp"
#include "a2w_fastlio_map/keyframe_database.hpp"
#include "a2w_fastlio_map/map_manager.hpp"

namespace a2w_fastlio_map
{
namespace
{

using a2w_fastlio_common::KeyFrame;
using a2w_fastlio_common::PointT;
using a2w_fastlio_common::ScanDescriptor;

KeyFrame keyframe(const std::uint64_t id, const float x = 0.0F)
{
  KeyFrame result;
  result.id = id;
  result.stamp_ns = static_cast<std::int64_t>(id + 1U) * 1'000'000'000LL;
  PointT point;
  point.x = x;
  result.body_cloud->push_back(point);
  return result;
}

ScanDescriptor descriptor(const float value)
{
  return ScanDescriptor{1U, 2U, {value, value + 1.0F}};
}

TEST(KeyFrameDatabase, RequiresOrderedIdsAndRejectsDuplicates)
{
  KeyFrameDatabase database;
  EXPECT_TRUE(database.add(keyframe(0U)));
  EXPECT_FALSE(database.add(keyframe(0U)));
  EXPECT_FALSE(database.add(keyframe(2U)));
  EXPECT_TRUE(database.add(keyframe(1U)));
  EXPECT_EQ(database.size(), 2U);
  ASSERT_TRUE(database.bounds());
  EXPECT_EQ(*database.bounds(), (std::pair<std::uint64_t, std::uint64_t>{0U, 1U}));
}

TEST(KeyFrameDatabase, ReturnsOwnedCloudCopies)
{
  KeyFrameDatabase database;
  ASSERT_TRUE(database.add(keyframe(0U, 3.0F)));
  auto first = database.get(0U);
  ASSERT_TRUE(first);
  first->body_cloud->front().x = 99.0F;

  const auto second = database.get(0U);
  ASSERT_TRUE(second);
  EXPECT_FLOAT_EQ(second->body_cloud->front().x, 3.0F);
}

TEST(DescriptorDatabase, DelegatesTopKThroughAbstractInterfaceAndKeepsRecords)
{
  DescriptorDatabase database;
  database.add(0U, descriptor(0.0F));
  database.add(1U, descriptor(5.0F));
  EXPECT_THROW(database.add(1U, descriptor(5.0F)), std::invalid_argument);

  a2w_fastlio_common::DescriptorIndex & abstract_index = database;
  const auto candidates = abstract_index.queryTopK(
    descriptor(0.1F), 2U, a2w_fastlio_common::CandidateFilter{});
  ASSERT_EQ(candidates.size(), 2U);
  EXPECT_EQ(candidates.front().keyframe_id, 0U);
  ASSERT_EQ(database.records().size(), 2U);
  EXPECT_EQ(database.records()[1].keyframe_id, 1U);
}

TEST(MapManager, EnforcesCountConsistencyAndProducesDeepImmutableSnapshot)
{
  MapManager manager;
  EXPECT_TRUE(manager.append(keyframe(0U, 4.0F), descriptor(1.0F)));
  EXPECT_FALSE(manager.append(keyframe(2U), descriptor(2.0F)));

  auto snapshot = manager.snapshot();
  ASSERT_EQ(snapshot.keyframes.size(), 1U);
  ASSERT_EQ(snapshot.descriptors.size(), 1U);
  snapshot.keyframes.front().body_cloud->front().x = 88.0F;
  EXPECT_FLOAT_EQ(manager.snapshot().keyframes.front().body_cloud->front().x, 4.0F);
}

TEST(MapManager, DescriptorShapeFailureDoesNotPartiallyAppendKeyframe)
{
  MapManager manager;
  ASSERT_TRUE(manager.append(keyframe(0U), descriptor(1.0F)));

  EXPECT_FALSE(manager.append(
    keyframe(1U), ScanDescriptor{1U, 3U, {1.0F, 2.0F, 3.0F}}));
  EXPECT_TRUE(manager.append(keyframe(1U), descriptor(2.0F)));
  EXPECT_EQ(manager.snapshot().keyframes.size(), 2U);
}

TEST(MapManager, ReadOnlyModeRejectsEveryMutation)
{
  MapManager manager{MapAccess::kReadOnly};
  EXPECT_THROW(manager.append(keyframe(0U), descriptor(0.0F)), std::logic_error);
  EXPECT_THROW(manager.setGlobalMap(keyframe(0U).body_cloud), std::logic_error);
}

}  // namespace
}  // namespace a2w_fastlio_map
