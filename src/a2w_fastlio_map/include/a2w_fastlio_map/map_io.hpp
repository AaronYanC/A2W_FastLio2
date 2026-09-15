#pragma once

#include <filesystem>
#include <string>

namespace a2w_fastlio_map
{

std::string sha256File(const std::filesystem::path & file);
std::filesystem::path confinedPath(
  const std::filesystem::path & root, const std::filesystem::path & relative);

}  // namespace a2w_fastlio_map
