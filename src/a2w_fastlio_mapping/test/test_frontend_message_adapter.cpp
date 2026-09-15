#include <gtest/gtest.h>

#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "a2w_fastlio_mapping/frontend_message_adapter.hpp"

namespace
{

nav_msgs::msg::Odometry makeOdometry()
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 12;
  odom.header.stamp.nanosec = 345'000'000U;
  odom.header.frame_id = "camera_init";
  odom.child_frame_id = "body";
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

sensor_msgs::msg::PointCloud2 makeBodyCloud()
{
  pcl::PointCloud<pcl::PointXYZINormal> cloud;
  pcl::PointXYZINormal point;
  point.x = 4.0F;
  point.y = 5.0F;
  point.z = 6.0F;
  point.intensity = 7.0F;
  point.normal_x = 8.0F;
  point.curvature = 9.0F;
  cloud.push_back(point);

  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 345'000'000U;
  message.header.frame_id = "body";
  return message;
}

TEST(FrontendMessageAdapter, ConvertsFastLioPoseStampAndPointFields)
{
  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    makeOdometry(), makeBodyCloud(), "camera_init", "body");

  ASSERT_TRUE(result.success) << result.error;
  EXPECT_EQ(result.frame.stamp_ns, 12'345'000'000LL);
  EXPECT_TRUE(result.frame.odom_pose.translation.isApprox(Eigen::Vector3d{1.0, 2.0, 3.0}));
  EXPECT_TRUE(result.frame.odom_pose.rotation.isApprox(Eigen::Quaterniond::Identity()));
  ASSERT_NE(result.frame.body_cloud, nullptr);
  ASSERT_EQ(result.frame.body_cloud->size(), 1U);
  const auto & point = result.frame.body_cloud->front();
  EXPECT_FLOAT_EQ(point.x, 4.0F);
  EXPECT_FLOAT_EQ(point.y, 5.0F);
  EXPECT_FLOAT_EQ(point.z, 6.0F);
  EXPECT_FLOAT_EQ(point.intensity, 7.0F);
}

TEST(FrontendMessageAdapter, OwnsConvertedCloudAfterSourceMutation)
{
  auto body_cloud = makeBodyCloud();
  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    makeOdometry(), body_cloud, "camera_init", "body");
  ASSERT_TRUE(result.success);

  body_cloud.data.assign(body_cloud.data.size(), 0U);

  ASSERT_EQ(result.frame.body_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(result.frame.body_cloud->front().x, 4.0F);
  EXPECT_FLOAT_EQ(result.frame.body_cloud->front().intensity, 7.0F);
}

TEST(FrontendMessageAdapter, RejectsTimestampMismatchOfOneNanosecond)
{
  auto cloud = makeBodyCloud();
  ++cloud.header.stamp.nanosec;

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    makeOdometry(), cloud, "camera_init", "body");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error.empty());
}

TEST(FrontendMessageAdapter, RejectsUnexpectedOdomParentFrame)
{
  auto odom = makeOdometry();
  odom.header.frame_id = "odom";

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    odom, makeBodyCloud(), "camera_init", "body");

  EXPECT_FALSE(result.success);
}

TEST(FrontendMessageAdapter, RejectsUnexpectedOdomChildFrame)
{
  auto odom = makeOdometry();
  odom.child_frame_id = "base_link";

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    odom, makeBodyCloud(), "camera_init", "body");

  EXPECT_FALSE(result.success);
}

TEST(FrontendMessageAdapter, RejectsUnexpectedCloudFrame)
{
  auto cloud = makeBodyCloud();
  cloud.header.frame_id = "lidar";

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    makeOdometry(), cloud, "camera_init", "body");

  EXPECT_FALSE(result.success);
}

TEST(FrontendMessageAdapter, EmptyExpectedFrameDisablesOnlyFrameChecks)
{
  auto odom = makeOdometry();
  auto cloud = makeBodyCloud();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  cloud.header.frame_id = "lidar";

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(odom, cloud, "", "");

  EXPECT_TRUE(result.success) << result.error;
}

TEST(FrontendMessageAdapter, RejectsEmptyPointCloud)
{
  auto cloud = makeBodyCloud();
  cloud.width = 0U;
  cloud.row_step = 0U;
  cloud.data.clear();

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    makeOdometry(), cloud, "camera_init", "body");

  EXPECT_FALSE(result.success);
}

TEST(FrontendMessageAdapter, RejectsInvalidOdometryPose)
{
  auto odom = makeOdometry();
  odom.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();

  const auto result = a2w_fastlio_mapping::makeFrontendFrame(
    odom, makeBodyCloud(), "camera_init", "body");

  EXPECT_FALSE(result.success);
}

}  // namespace
