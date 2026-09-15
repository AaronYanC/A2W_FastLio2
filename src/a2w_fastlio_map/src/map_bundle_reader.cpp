#include "a2w_fastlio_map/map_bundle_reader.hpp"

#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "a2w_fastlio_common/registration.hpp"
#include "a2w_fastlio_map/map_io.hpp"
#include "a2w_fastlio_map/map_manifest.hpp"

namespace a2w_fastlio_map
{
namespace
{

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream{line};
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

a2w_fastlio_common::Pose3d pose(const std::vector<std::string> & fields, const std::size_t offset)
{
  if (fields.size() < offset + 7U) {
    throw std::runtime_error{"pose CSV row has too few fields"};
  }
  a2w_fastlio_common::Pose3d result;
  result.translation = Eigen::Vector3d{
    std::stod(fields[offset]), std::stod(fields[offset + 1U]), std::stod(fields[offset + 2U])};
  result.rotation = Eigen::Quaterniond{
    std::stod(fields[offset + 6U]), std::stod(fields[offset + 3U]),
    std::stod(fields[offset + 4U]), std::stod(fields[offset + 5U])};
  if (!a2w_fastlio_common::isFinitePose(result) || result.rotation.squaredNorm() < 1e-12) {
    throw std::runtime_error{"pose CSV contains invalid pose"};
  }
  result.rotation.normalize();
  return result;
}

std::string readText(const std::filesystem::path & path)
{
  std::ifstream stream{path, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"cannot read text file: " + path.string()};
  }
  return std::string{std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

}  // namespace

MapBundle MapBundleReader::read(const std::filesystem::path & root, const ReadMode mode) const
{
  const auto canonical_root = std::filesystem::weakly_canonical(root);
  if (!std::filesystem::is_directory(canonical_root)) {
    throw std::runtime_error{"Map Bundle root is not a directory"};
  }
  const auto manifest = MapManifest::read(canonical_root);
  const auto verification = manifest.verify(canonical_root);
  if (!verification.valid) {
    throw std::runtime_error{"Map Bundle manifest verification failed: " +
      verification.errors.front()};
  }

  MapBundle bundle;
  bundle.root = canonical_root;
  bundle.mode = mode;
  const auto metadata = YAML::LoadFile(confinedPath(canonical_root, "metadata.yaml").string());
  auto & output = bundle.data.metadata;
  output.schema = metadata["schema"].as<std::string>();
  if (output.schema != kMapBundleSchema) {
    throw std::runtime_error{"unsupported Map Bundle schema"};
  }
  output.bundle_uuid = metadata["bundle_uuid"].as<std::string>();
  output.created_utc = metadata["created_utc"].as<std::string>();
  output.map_frame = metadata["map_frame"].as<std::string>();
  output.odom_frame = metadata["odom_frame"].as<std::string>();
  output.tracking_frame = metadata["tracking_frame"].as<std::string>();
  output.point_type = metadata["point_type"].as<std::string>();
  output.global_map_leaf_m = metadata["global_map_leaf_m"].as<double>();
  output.frontend_revision = metadata["frontend_revision"].as<std::string>();
  output.dependency_revisions =
    metadata["dependency_revisions"].as<std::map<std::string, std::string>>();
  output.config_hashes = metadata["config_hashes"].as<std::map<std::string, std::string>>();
  output.creation_status = metadata["creation_status"].as<std::string>();
  output.hardware_validation_status = metadata["hardware_validation_status"].as<std::string>();
  const auto keyframe_count = metadata["keyframe_count"].as<std::size_t>();
  const auto descriptor_count = metadata["descriptor_count"].as<std::size_t>();
  const auto descriptor_rings = metadata["descriptor_rings"].as<std::size_t>();
  const auto descriptor_sectors = metadata["descriptor_sectors"].as<std::size_t>();
  if (output.bundle_uuid.empty() || output.map_frame.empty() || output.odom_frame.empty() ||
    output.tracking_frame.empty() || output.point_type != "PointXYZI" ||
    keyframe_count == 0U || keyframe_count != descriptor_count)
  {
    throw std::runtime_error{"invalid Map Bundle metadata"};
  }

  std::ifstream index{confinedPath(canonical_root, "keyframes/index.csv"), std::ios::binary};
  std::string line;
  std::getline(index, line);
  if (line != "id,stamp_ns,cloud,tx,ty,tz,qx,qy,qz,qw") {
    throw std::runtime_error{"invalid keyframe index header"};
  }
  while (std::getline(index, line)) {
    const auto fields = split(line);
    if (fields.size() != 10U) {
      throw std::runtime_error{"invalid keyframe index row"};
    }
    a2w_fastlio_common::KeyFrame frame;
    frame.id = std::stoull(fields[0]);
    frame.stamp_ns = std::stoll(fields[1]);
    if (frame.id != bundle.data.keyframes.size() || frame.stamp_ns < 0) {
      throw std::runtime_error{"invalid keyframe ID or timestamp"};
    }
    frame.odom_pose = pose(fields, 3U);
    frame.body_cloud = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
    if (pcl::io::loadPCDFile(
      confinedPath(canonical_root, fields[2]).string(), *frame.body_cloud) != 0 ||
      frame.body_cloud->empty())
    {
      throw std::runtime_error{"cannot read keyframe PCD"};
    }
    bundle.data.keyframes.push_back(std::move(frame));
  }
  if (bundle.data.keyframes.size() != keyframe_count) {
    throw std::runtime_error{"keyframe count mismatch"};
  }

  std::ifstream poses{confinedPath(canonical_root, "optimized_poses.csv"), std::ios::binary};
  std::getline(poses, line);
  if (line != "id,tx,ty,tz,qx,qy,qz,qw") {
    throw std::runtime_error{"invalid optimized pose header"};
  }
  std::size_t pose_count = 0U;
  while (std::getline(poses, line)) {
    const auto fields = split(line);
    if (fields.size() != 8U || std::stoull(fields[0]) != pose_count ||
      pose_count >= bundle.data.keyframes.size())
    {
      throw std::runtime_error{"invalid optimized pose row"};
    }
    bundle.data.keyframes[pose_count].optimized_pose = pose(fields, 1U);
    ++pose_count;
  }
  if (pose_count != keyframe_count) {
    throw std::runtime_error{"optimized pose count mismatch"};
  }

  std::ifstream descriptors{
    confinedPath(canonical_root, "descriptors/scan_context.bin"), std::ios::binary};
  std::array<char, 8U> magic{};
  descriptors.read(magic.data(), magic.size());
  const std::array<char, 8U> expected{'A', '2', 'W', 'S', 'C', 'V', '1', '\0'};
  std::uint64_t count = 0U;
  std::uint64_t rings = 0U;
  std::uint64_t sectors = 0U;
  descriptors.read(reinterpret_cast<char *>(&count), sizeof(count));
  descriptors.read(reinterpret_cast<char *>(&rings), sizeof(rings));
  descriptors.read(reinterpret_cast<char *>(&sectors), sizeof(sectors));
  if (!descriptors || magic != expected || count != descriptor_count ||
    rings != descriptor_rings || sectors != descriptor_sectors || rings == 0U || sectors == 0U)
  {
    throw std::runtime_error{"invalid descriptor header"};
  }
  for (std::uint64_t index_value = 0U; index_value < count; ++index_value) {
    std::uint64_t id = 0U;
    std::vector<float> values(static_cast<std::size_t>(rings * sectors));
    descriptors.read(reinterpret_cast<char *>(&id), sizeof(id));
    descriptors.read(
      reinterpret_cast<char *>(values.data()),
      static_cast<std::streamsize>(values.size() * sizeof(float)));
    if (!descriptors || id != index_value) {
      throw std::runtime_error{"invalid descriptor record"};
    }
    bundle.data.descriptors.push_back({
      id, a2w_fastlio_common::ScanDescriptor{
        static_cast<std::size_t>(rings), static_cast<std::size_t>(sectors), std::move(values)}});
  }

  bundle.data.global_map = a2w_fastlio_common::CloudPtr{new a2w_fastlio_common::Cloud{}};
  if (pcl::io::loadPCDFile(
    confinedPath(canonical_root, "global_map.pcd").string(), *bundle.data.global_map) != 0 ||
    bundle.data.global_map->empty())
  {
    throw std::runtime_error{"cannot read global map PCD"};
  }
  for (const auto & name : {
      "mapping_effective.yaml", "scan_context.yaml", "registration.yaml"})
  {
    bundle.data.config_snapshots.emplace(
      name, readText(confinedPath(canonical_root, std::filesystem::path{"config"} / name)));
    const auto hash = sha256File(
      confinedPath(canonical_root, std::filesystem::path{"config"} / name));
    if (output.config_hashes.at(name) != hash) {
      throw std::runtime_error{"configuration hash mismatch"};
    }
  }
  return bundle;
}

}  // namespace a2w_fastlio_map
