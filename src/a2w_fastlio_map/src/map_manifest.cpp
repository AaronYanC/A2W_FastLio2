#include "a2w_fastlio_map/map_manifest.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <set>
#include <stdexcept>
#include <utility>

#include "a2w_fastlio_map/map_io.hpp"

namespace a2w_fastlio_map
{

MapManifest::MapManifest(std::vector<ManifestEntry> entries) : entries_{std::move(entries)} {}

MapManifest MapManifest::build(
  const std::filesystem::path & root,
  const std::vector<std::filesystem::path> & files)
{
  std::set<std::string> unique;
  std::vector<ManifestEntry> entries;
  entries.reserve(files.size());
  for (const auto & relative : files) {
    const auto portable = relative.generic_string();
    if (!unique.insert(portable).second) {
      throw std::invalid_argument{"duplicate manifest path: " + portable};
    }
    entries.push_back({relative.lexically_normal(), sha256File(confinedPath(root, relative))});
  }
  std::sort(entries.begin(), entries.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.relative_path.generic_string() < rhs.relative_path.generic_string();
    });
  return MapManifest{std::move(entries)};
}

void MapManifest::write(const std::filesystem::path & root) const
{
  std::ofstream stream{confinedPath(root, "manifest.sha256"), std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"cannot write manifest.sha256"};
  }
  for (const auto & entry : entries_) {
    stream << entry.sha256 << "  " << entry.relative_path.generic_string() << '\n';
  }
  if (!stream) {
    throw std::runtime_error{"cannot complete manifest.sha256"};
  }
}

MapManifest MapManifest::read(const std::filesystem::path & root)
{
  std::ifstream stream{confinedPath(root, "manifest.sha256"), std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"cannot read manifest.sha256"};
  }
  std::set<std::string> unique;
  std::vector<ManifestEntry> entries;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.size() < 67U || line.substr(64U, 2U) != "  " ||
      !std::all_of(line.begin(), line.begin() + 64, [](const unsigned char value) {
        return std::isxdigit(value) != 0;
      }))
    {
      throw std::runtime_error{"malformed manifest line"};
    }
    const auto relative = line.substr(66U);
    if (relative.empty() || !unique.insert(relative).second) {
      throw std::runtime_error{"empty or duplicate manifest path"};
    }
    confinedPath(root, relative);
    entries.push_back({relative, line.substr(0U, 64U)});
  }
  if (entries.empty()) {
    throw std::runtime_error{"manifest contains no entries"};
  }
  if (!std::is_sorted(entries.begin(), entries.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.relative_path.generic_string() < rhs.relative_path.generic_string();
    }))
  {
    throw std::runtime_error{"manifest entries are not sorted"};
  }
  return MapManifest{std::move(entries)};
}

ManifestVerification MapManifest::verify(const std::filesystem::path & root) const
{
  ManifestVerification result;
  for (const auto & entry : entries_) {
    const auto portable = entry.relative_path.generic_string();
    try {
      const auto path = confinedPath(root, entry.relative_path);
      if (!std::filesystem::is_regular_file(path)) {
        result.errors.push_back("missing:" + portable);
      } else if (sha256File(path) != entry.sha256) {
        result.errors.push_back("hash_mismatch:" + portable);
      }
    } catch (const std::exception &) {
      result.errors.push_back("invalid_path:" + portable);
    }
  }
  result.valid = result.errors.empty();
  return result;
}

const std::vector<ManifestEntry> & MapManifest::entries() const noexcept
{
  return entries_;
}

}  // namespace a2w_fastlio_map
