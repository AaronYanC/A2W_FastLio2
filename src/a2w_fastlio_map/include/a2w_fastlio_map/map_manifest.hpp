#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace a2w_fastlio_map
{

struct ManifestEntry
{
  std::filesystem::path relative_path;
  std::string sha256;
};

struct ManifestVerification
{
  bool valid{false};
  std::vector<std::string> errors;
};

class MapManifest
{
public:
  static MapManifest build(
    const std::filesystem::path & root,
    const std::vector<std::filesystem::path> & files);
  static MapManifest read(const std::filesystem::path & root);

  void write(const std::filesystem::path & root) const;
  ManifestVerification verify(const std::filesystem::path & root) const;
  const std::vector<ManifestEntry> & entries() const noexcept;

private:
  explicit MapManifest(std::vector<ManifestEntry> entries);
  std::vector<ManifestEntry> entries_;
};

}  // namespace a2w_fastlio_map
