#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_map/map_bundle_writer.hpp"
#include "a2w_fastlio_map/map_manifest.hpp"

namespace a2w_fastlio_map
{
namespace
{

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    auto pattern = (std::filesystem::temp_directory_path() / "a2w-bundle-fail-XXXXXX").string();
    root_ = ::mkdtemp(pattern.data());
  }
  ~TemporaryDirectory() {std::filesystem::remove_all(root_);}
  std::filesystem::path root() const {return root_;}
private:
  std::filesystem::path root_;
};

MapBundleData validData(const std::string & uuid = "valid-uuid")
{
  MapBundleData data;
  data.metadata.bundle_uuid = uuid;
  data.metadata.created_utc = "2026-09-15T00:00:00Z";
  data.metadata.frontend_revision = "frontend";
  a2w_fastlio_common::KeyFrame frame;
  frame.stamp_ns = 1;
  frame.body_cloud->push_back(a2w_fastlio_common::PointT{});
  data.keyframes.push_back(frame);
  data.descriptors.push_back({0U, a2w_fastlio_common::ScanDescriptor{1U, 1U, {1.0F}}});
  data.global_map->push_back(a2w_fastlio_common::PointT{});
  data.config_snapshots = {
    {"mapping_effective.yaml", "a: 1\n"},
    {"scan_context.yaml", "a: 1\n"},
    {"registration.yaml", "a: 1\n"},
  };
  return data;
}

void refreshManifest(const std::filesystem::path & root)
{
  std::vector<std::filesystem::path> files;
  const auto existing = MapManifest::read(root);
  for (const auto & entry : existing.entries()) {
    files.push_back(entry.relative_path);
  }
  MapManifest::build(root, files).write(root);
}

TEST(MapBundleFailures, RejectsWrongSchemaDimensionMismatchAndNanPose)
{
  TemporaryDirectory directory;
  auto data = validData();
  data.metadata.schema = "future";
  EXPECT_FALSE(MapBundleWriter{}.write(directory.root() / "wrong", data).success);

  data = validData();
  data.descriptors.push_back({1U, a2w_fastlio_common::ScanDescriptor{1U, 2U, {1.0F, 2.0F}}});
  EXPECT_FALSE(MapBundleWriter{}.write(directory.root() / "count", data).success);

  data = validData();
  data.keyframes[0].optimized_pose.translation.x() = NAN;
  EXPECT_FALSE(MapBundleWriter{}.write(directory.root() / "nan", data).success);
}

TEST(MapBundleFailures, ReaderRejectsCorruptHashAndMissingCloud)
{
  TemporaryDirectory directory;
  const auto destination = directory.root() / "map";
  ASSERT_TRUE(MapBundleWriter{}.write(destination, validData()).success);
  {
    std::ofstream stream{destination / "metadata.yaml", std::ios::app};
    stream << "# corrupt\n";
  }
  EXPECT_THROW(MapBundleReader{}.read(destination), std::runtime_error);

  const auto second = directory.root() / "missing";
  ASSERT_TRUE(MapBundleWriter{}.write(second, validData()).success);
  std::filesystem::remove(second / "keyframes/clouds/000000.pcd");
  EXPECT_THROW(MapBundleReader{}.read(second), std::runtime_error);
}

TEST(MapBundleFailures, ExistingDestinationSurvivesInjectedReplacementFailure)
{
  TemporaryDirectory directory;
  const auto destination = directory.root() / "map";
  ASSERT_TRUE(MapBundleWriter{}.write(destination, validData("old")).success);
  const auto failed = MapBundleWriter{BundleFailurePoint::kAfterBackupRename}.write(
    destination, validData("new"));
  EXPECT_FALSE(failed.success);
  EXPECT_EQ(MapBundleReader{}.read(destination).data.metadata.bundle_uuid, "old");

  const auto staging_failed = MapBundleWriter{BundleFailurePoint::kBeforeValidation}.write(
    destination, validData("newer"));
  EXPECT_FALSE(staging_failed.success);
  EXPECT_EQ(MapBundleReader{}.read(destination).data.metadata.bundle_uuid, "old");
}

TEST(MapBundleFailures, ReaderRejectsWrongSchemaAndUnreadablePcdAfterValidHash)
{
  TemporaryDirectory directory;
  const auto schema = directory.root() / "schema";
  ASSERT_TRUE(MapBundleWriter{}.write(schema, validData()).success);
  {
    auto metadata = YAML::LoadFile((schema / "metadata.yaml").string());
    metadata["schema"] = "future_schema";
    std::ofstream{schema / "metadata.yaml"} << metadata;
  }
  refreshManifest(schema);
  EXPECT_THROW(MapBundleReader{}.read(schema), std::runtime_error);

  const auto pcd = directory.root() / "pcd";
  ASSERT_TRUE(MapBundleWriter{}.write(pcd, validData()).success);
  std::ofstream{pcd / "keyframes/clouds/000000.pcd", std::ios::binary} << "not a PCD";
  refreshManifest(pcd);
  EXPECT_THROW(MapBundleReader{}.read(pcd), std::runtime_error);
}

TEST(MapBundleFailures, DoesNotDeleteUnrelatedStaleStagingDirectory)
{
  TemporaryDirectory directory;
  const auto stale = directory.root() / ".map.staging-stale";
  std::filesystem::create_directory(stale);
  std::ofstream{stale / "owned-by-other-operation"} << "keep";
  ASSERT_TRUE(MapBundleWriter{}.write(directory.root() / "map", validData()).success);
  EXPECT_TRUE(std::filesystem::exists(stale / "owned-by-other-operation"));
}

}  // namespace
}  // namespace a2w_fastlio_map
