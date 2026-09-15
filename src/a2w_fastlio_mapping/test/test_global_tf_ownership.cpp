#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/global_tf_ownership.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

GlobalTfOwnerState owner(
  std::string id, const std::int64_t conflict_window_ns = 2'000'000'000LL)
{
  return GlobalTfOwnerState{
    GlobalTfOwnerConfig{
      std::move(id), "mapping", "map", "camera_init", conflict_window_ns}};
}

TEST(GlobalTfOwnership, AllowsSingleOwnerAfterObservationWindow)
{
  auto state = owner("mapping-a", 100);
  state.start(1'000);

  EXPECT_FALSE(state.mayPublish(1'099));
  EXPECT_TRUE(state.mayPublish(1'100));
  EXPECT_FALSE(state.conflicted(1'100));
}

TEST(GlobalTfOwnership, ForeignMatchingOwnerDisablesPublishing)
{
  auto state = owner("mapping-a", 100);
  state.start(1'000);
  ASSERT_TRUE(state.mayPublish(1'100));

  const GlobalTfOwnerObservation foreign{
    1'110, "localization-b", "localization", "map", "camera_init", true};
  EXPECT_TRUE(state.observe(foreign, 1'110));
  EXPECT_TRUE(state.conflicted(1'110));
  EXPECT_FALSE(state.mayPublish(1'110));
  EXPECT_EQ(state.faultReason(1'110), "foreign_global_tf_owner_active");
}

TEST(GlobalTfOwnership, IgnoresSelfInactiveAndDifferentTransform)
{
  auto state = owner("mapping-a", 100);
  state.start(1'000);

  EXPECT_FALSE(state.observe(
    {1'010, "mapping-a", "mapping", "map", "camera_init", true}, 1'010));
  EXPECT_FALSE(state.observe(
    {1'020, "other", "localization", "world", "camera_init", true}, 1'020));
  EXPECT_FALSE(state.observe(
    {1'030, "other", "localization", "map", "camera_init", false}, 1'030));
  EXPECT_TRUE(state.mayPublish(1'100));
}

TEST(GlobalTfOwnership, ConflictRemainsLatchedForProcessLifetime)
{
  auto state = owner("mapping-a", 100);
  state.start(1'000);
  ASSERT_TRUE(state.observe(
    {1'010, "other", "mapping", "map", "camera_init", true}, 1'010));

  EXPECT_FALSE(state.mayPublish(100'000));
  EXPECT_TRUE(state.conflicted(100'000));
}

TEST(GlobalTfOwnership, RejectsInvalidConfigurationAndUseBeforeStart)
{
  EXPECT_THROW(
    GlobalTfOwnerState(GlobalTfOwnerConfig{"", "mapping", "map", "camera_init", 1}),
    std::invalid_argument);
  EXPECT_THROW(
    GlobalTfOwnerState(GlobalTfOwnerConfig{"a", "mapping", "map", "map", 1}),
    std::invalid_argument);
  EXPECT_THROW(
    GlobalTfOwnerState(GlobalTfOwnerConfig{"a", "mapping", "map", "camera_init", 0}),
    std::invalid_argument);

  auto state = owner("mapping-a");
  EXPECT_FALSE(state.mayPublish(1));
  EXPECT_THROW(state.start(-1), std::invalid_argument);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
