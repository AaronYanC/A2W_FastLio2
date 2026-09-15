#include <cstdlib>
#include <filesystem>

#include <gtest/gtest.h>

#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_map/map_bundle_writer.hpp"

namespace a2w_fastlio_map
{
namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    auto pattern = (std::filesystem::temp_directory_path() / "a2w-bundle-XXXXXX").string();
    root_ = ::mkdtemp(pattern.data());
  }
  ~TemporaryDirectory() {std::filesystem::remove_all(root_);}
  std::filesystem::path root() const {return root_;}
private:
  std::filesystem::path root_;
};

MapBundleData sampleBundle()
{
  MapBundleData data;
  data.metadata.bundle_uuid = "00000000-0000-4000-8000-000000000001";
  data.metadata.created_utc = "2026-09-15T00:00:00Z";
  data.metadata.frontend_revision = "protected-fast-lio-revision";
  data.metadata.dependency_revisions = {{"scan_context", "abc"}, {"quatro", "def"}};
  for (std::uint64_t id = 0U; id < 2U; ++id) {
    a2w_fastlio_common::KeyFrame frame;
    frame.id = id;
    frame.stamp_ns = static_cast<std::int64_t>(id + 1U) * 1000;
    frame.odom_pose.translation.x() = static_cast<double>(id);
    frame.optimized_pose.translation.x() = static_cast<double>(id) + 0.25;
    a2w_fastlio_common::PointT point;
    point.x = static_cast<float>(id) + 0.5F;
    point.intensity = static_cast<float>(id + 10U);
    frame.body_cloud->push_back(point);
    data.keyframes.push_back(frame);
    data.descriptors.push_back({
      id, a2w_fastlio_common::ScanDescriptor{2U, 2U,
        {static_cast<float>(id), 1.0F, 2.0F, 3.0F}}});
    data.global_map->push_back(point);
  }
  data.config_snapshots = {
    {"mapping_effective.yaml", "mapping: true\n"},
    {"scan_context.yaml", "rings: 2\n"},
    {"registration.yaml", "fine: true\n"},
  };
  return data;
}

TEST(MapBundleRoundTrip, PreservesMetadataPosesCloudsDescriptorsAndConfigs)
{
  TemporaryDirectory directory;
  const auto destination = directory.root() / "factory map";
  const auto expected = sampleBundle();
  const auto result = MapBundleWriter{}.write(destination, expected);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.bundle_uuid, expected.metadata.bundle_uuid);
  EXPECT_EQ(result.keyframe_count, 2U);
  EXPECT_EQ(result.resolved_path, std::filesystem::weakly_canonical(destination));

  const auto loaded = MapBundleReader{}.read(destination);
  EXPECT_EQ(loaded.mode, ReadMode::kReadOnly);
  EXPECT_EQ(loaded.data.metadata.schema, kMapBundleSchema);
  EXPECT_EQ(loaded.data.metadata.bundle_uuid, expected.metadata.bundle_uuid);
  EXPECT_EQ(loaded.data.metadata.hardware_validation_status, "pending");
  EXPECT_EQ(loaded.data.metadata.dependency_revisions, expected.metadata.dependency_revisions);
  ASSERT_EQ(loaded.data.keyframes.size(), 2U);
  EXPECT_EQ(loaded.data.keyframes[1].stamp_ns, 2000);
  EXPECT_DOUBLE_EQ(loaded.data.keyframes[1].optimized_pose.translation.x(), 1.25);
  EXPECT_FLOAT_EQ(loaded.data.keyframes[1].body_cloud->front().intensity, 11.0F);
  ASSERT_EQ(loaded.data.descriptors.size(), 2U);
  EXPECT_EQ(loaded.data.descriptors[0].descriptor.rings(), 2U);
  EXPECT_EQ(loaded.data.descriptors[1].descriptor.values()[0], 1.0F);
  EXPECT_EQ(loaded.data.global_map->size(), 2U);
  EXPECT_EQ(loaded.data.config_snapshots, expected.config_snapshots);
}

}  // namespace
}  // namespace a2w_fastlio_map
