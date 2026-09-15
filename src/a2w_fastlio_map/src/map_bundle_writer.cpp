#include "a2w_fastlio_map/map_bundle_writer.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include "a2w_fastlio_common/registration.hpp"
#include "a2w_fastlio_map/map_bundle_reader.hpp"
#include "a2w_fastlio_map/map_io.hpp"
#include "a2w_fastlio_map/map_manifest.hpp"

namespace a2w_fastlio_map
{
namespace
{

std::string token()
{
  return std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
}

void validate(const MapBundleData & data)
{
  if (data.metadata.schema != kMapBundleSchema || data.metadata.bundle_uuid.empty() ||
    data.metadata.created_utc.empty() || data.metadata.map_frame.empty() ||
    data.metadata.odom_frame.empty() || data.metadata.tracking_frame.empty() ||
    data.metadata.point_type != "PointXYZI" || data.metadata.creation_status.empty() ||
    data.metadata.hardware_validation_status.empty() || data.keyframes.empty() ||
    data.keyframes.size() != data.descriptors.size() || !data.global_map ||
    data.global_map->empty())
  {
    throw std::invalid_argument{"invalid Map Bundle metadata or counts"};
  }
  const std::set<std::string> required{
    "mapping_effective.yaml", "scan_context.yaml", "registration.yaml"};
  std::set<std::string> supplied;
  for (const auto & item : data.config_snapshots) {
    supplied.insert(item.first);
  }
  if (supplied != required) {
    throw std::invalid_argument{"Map Bundle requires exactly three configuration snapshots"};
  }
  const auto rings = data.descriptors.front().descriptor.rings();
  const auto sectors = data.descriptors.front().descriptor.sectors();
  for (std::size_t index = 0U; index < data.keyframes.size(); ++index) {
    const auto & frame = data.keyframes[index];
    const auto & descriptor = data.descriptors[index];
    if (frame.id != index || descriptor.keyframe_id != index || frame.stamp_ns < 0 ||
      !frame.body_cloud || frame.body_cloud->empty() ||
      !a2w_fastlio_common::isFinitePose(frame.odom_pose) ||
      !a2w_fastlio_common::isFinitePose(frame.optimized_pose) ||
      descriptor.descriptor.rings() != rings || descriptor.descriptor.sectors() != sectors)
    {
      throw std::invalid_argument{"invalid keyframe, pose, or descriptor sequence"};
    }
  }
}

void emitPose(std::ostream & stream, const a2w_fastlio_common::Pose3d & pose)
{
  stream << ',' << std::setprecision(17) << pose.translation.x() << ',' << pose.translation.y() <<
    ',' << pose.translation.z() << ',' << pose.rotation.x() << ',' << pose.rotation.y() << ',' <<
    pose.rotation.z() << ',' << pose.rotation.w();
}

void writeData(const std::filesystem::path & root, const MapBundleData & data)
{
  std::filesystem::create_directories(root / "keyframes/clouds");
  std::filesystem::create_directories(root / "descriptors");
  std::filesystem::create_directories(root / "config");
  std::map<std::string, std::string> config_hashes;
  for (const auto & config : data.config_snapshots) {
    const auto path = root / "config" / config.first;
    std::ofstream{path, std::ios::binary} << config.second;
    config_hashes.emplace(config.first, sha256File(path));
  }

  YAML::Emitter metadata;
  metadata << YAML::BeginMap << YAML::Key << "schema" << YAML::Value << data.metadata.schema
           << YAML::Key << "bundle_uuid" << YAML::Value << data.metadata.bundle_uuid
           << YAML::Key << "created_utc" << YAML::Value << data.metadata.created_utc
           << YAML::Key << "map_frame" << YAML::Value << data.metadata.map_frame
           << YAML::Key << "odom_frame" << YAML::Value << data.metadata.odom_frame
           << YAML::Key << "tracking_frame" << YAML::Value << data.metadata.tracking_frame
           << YAML::Key << "point_type" << YAML::Value << data.metadata.point_type
           << YAML::Key << "keyframe_count" << YAML::Value << data.keyframes.size()
           << YAML::Key << "descriptor_count" << YAML::Value << data.descriptors.size()
           << YAML::Key << "descriptor_rings" << YAML::Value
           << data.descriptors.front().descriptor.rings()
           << YAML::Key << "descriptor_sectors" << YAML::Value
           << data.descriptors.front().descriptor.sectors()
           << YAML::Key << "global_map_leaf_m" << YAML::Value
           << data.metadata.global_map_leaf_m
           << YAML::Key << "frontend_revision" << YAML::Value
           << data.metadata.frontend_revision
           << YAML::Key << "dependency_revisions" << YAML::Value
           << data.metadata.dependency_revisions
           << YAML::Key << "config_hashes" << YAML::Value << config_hashes
           << YAML::Key << "creation_status" << YAML::Value
           << data.metadata.creation_status
           << YAML::Key << "hardware_validation_status" << YAML::Value
           << data.metadata.hardware_validation_status << YAML::EndMap;
  std::ofstream{root / "metadata.yaml", std::ios::binary} << metadata.c_str() << '\n';

  std::ofstream index{root / "keyframes/index.csv", std::ios::binary};
  std::ofstream poses{root / "optimized_poses.csv", std::ios::binary};
  index << "id,stamp_ns,cloud,tx,ty,tz,qx,qy,qz,qw\n";
  poses << "id,tx,ty,tz,qx,qy,qz,qw\n";
  for (const auto & frame : data.keyframes) {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << frame.id << ".pcd";
    const auto relative = std::filesystem::path{"keyframes/clouds"} / filename.str();
    if (pcl::io::savePCDFileBinary((root / relative).string(), *frame.body_cloud) != 0) {
      throw std::runtime_error{"cannot write keyframe PCD"};
    }
    index << frame.id << ',' << frame.stamp_ns << ',' << relative.generic_string();
    emitPose(index, frame.odom_pose);
    index << '\n';
    poses << frame.id;
    emitPose(poses, frame.optimized_pose);
    poses << '\n';
  }
  index.close();
  poses.close();
  if (!index || !poses) {
    throw std::runtime_error{"cannot complete keyframe or pose index"};
  }

  std::ofstream descriptors{root / "descriptors/scan_context.bin", std::ios::binary};
  const char magic[8] = {'A', '2', 'W', 'S', 'C', 'V', '1', '\0'};
  descriptors.write(magic, sizeof(magic));
  const std::uint64_t count = data.descriptors.size();
  const std::uint64_t rings = data.descriptors.front().descriptor.rings();
  const std::uint64_t sectors = data.descriptors.front().descriptor.sectors();
  descriptors.write(reinterpret_cast<const char *>(&count), sizeof(count));
  descriptors.write(reinterpret_cast<const char *>(&rings), sizeof(rings));
  descriptors.write(reinterpret_cast<const char *>(&sectors), sizeof(sectors));
  for (const auto & record : data.descriptors) {
    descriptors.write(
      reinterpret_cast<const char *>(&record.keyframe_id), sizeof(record.keyframe_id));
    descriptors.write(
      reinterpret_cast<const char *>(record.descriptor.values().data()),
      static_cast<std::streamsize>(record.descriptor.values().size() * sizeof(float)));
  }
  if (!descriptors) {
    throw std::runtime_error{"cannot write descriptors"};
  }
  descriptors.close();
  if (!descriptors) {
    throw std::runtime_error{"cannot complete descriptors"};
  }
  if (pcl::io::savePCDFileBinary((root / "global_map.pcd").string(), *data.global_map) != 0) {
    throw std::runtime_error{"cannot write global map PCD"};
  }
  std::vector<std::filesystem::path> files{
    "metadata.yaml", "optimized_poses.csv", "keyframes/index.csv",
    "descriptors/scan_context.bin", "global_map.pcd",
    "config/mapping_effective.yaml", "config/scan_context.yaml", "config/registration.yaml"};
  for (const auto & frame : data.keyframes) {
    std::ostringstream filename;
    filename << "keyframes/clouds/" << std::setw(6) << std::setfill('0') << frame.id << ".pcd";
    files.emplace_back(filename.str());
  }
  auto manifest = MapManifest::build(root, files);
  manifest.write(root);
}

}  // namespace

MapBundleWriter::MapBundleWriter(const BundleFailurePoint failure_point)
: failure_point_{failure_point} {}

BundleWriteResult MapBundleWriter::write(
  const std::filesystem::path & destination, const MapBundleData & data) const
{
  BundleWriteResult result;
  result.bundle_uuid = data.metadata.bundle_uuid;
  result.keyframe_count = data.keyframes.size();
  std::filesystem::path staging;
  std::filesystem::path backup;
  bool destination_was_backed_up = false;
  try {
    validate(data);
    const auto parent = std::filesystem::absolute(destination).parent_path();
    if (destination.filename().empty()) {
      throw std::invalid_argument{"Map Bundle destination requires a filename"};
    }
    std::filesystem::create_directories(parent);
    const auto suffix = token();
    staging = parent / ("." + destination.filename().string() + ".staging-" + suffix);
    backup = parent / ("." + destination.filename().string() + ".backup-" + suffix);
    std::filesystem::create_directory(staging);
    writeData(staging, data);
    if (failure_point_ == BundleFailurePoint::kBeforeValidation) {
      throw std::runtime_error{"injected failure before validation"};
    }
    MapBundleReader{}.read(staging);
    const auto absolute_destination = std::filesystem::absolute(destination);
    if (std::filesystem::exists(absolute_destination)) {
      std::filesystem::rename(absolute_destination, backup);
      destination_was_backed_up = true;
    }
    if (failure_point_ == BundleFailurePoint::kAfterBackupRename) {
      throw std::runtime_error{"injected failure after backup rename"};
    }
    std::filesystem::rename(staging, absolute_destination);
    if (destination_was_backed_up) {
      std::filesystem::remove_all(backup);
    }
    result.success = true;
    result.message = "ok";
    result.resolved_path = std::filesystem::weakly_canonical(absolute_destination);
  } catch (const std::exception & error) {
    result.message = error.what();
    std::error_code ignored;
    if (!staging.empty()) {
      std::filesystem::remove_all(staging, ignored);
    }
    const auto absolute_destination = std::filesystem::absolute(destination);
    if (destination_was_backed_up && !std::filesystem::exists(absolute_destination) &&
      std::filesystem::exists(backup))
    {
      std::filesystem::rename(backup, absolute_destination, ignored);
    }
  }
  return result;
}

}  // namespace a2w_fastlio_map
