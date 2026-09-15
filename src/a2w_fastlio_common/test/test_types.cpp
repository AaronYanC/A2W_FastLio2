#include <gtest/gtest.h>

#include "a2w_fastlio_common/types.hpp"

TEST(KeyFrameTypes, DefaultsPoseToIdentityAndDescriptorToEmpty)
{
  a2w_fastlio_common::KeyFrame keyframe;

  EXPECT_EQ(keyframe.id, 0U);
  EXPECT_EQ(keyframe.stamp_ns, 0);
  EXPECT_TRUE(keyframe.odom_pose.translation.isZero());
  EXPECT_TRUE(keyframe.odom_pose.rotation.isApprox(Eigen::Quaterniond::Identity()));
  EXPECT_TRUE(keyframe.optimized_pose.translation.isZero());
  EXPECT_TRUE(keyframe.optimized_pose.rotation.isApprox(Eigen::Quaterniond::Identity()));
  ASSERT_NE(keyframe.body_cloud, nullptr);
  EXPECT_TRUE(keyframe.body_cloud->empty());
  EXPECT_TRUE(keyframe.scan_context_descriptor.empty());
}
