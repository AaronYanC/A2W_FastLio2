#include "a2w_fastlio_map/map_io.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

#include <openssl/evp.h>

namespace a2w_fastlio_map
{

std::string sha256File(const std::filesystem::path & file)
{
  std::ifstream stream{file, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"cannot open file for SHA-256: " + file.string()};
  }
  using Context = std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  Context context{EVP_MD_CTX_new(), EVP_MD_CTX_free};
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error{"cannot initialize SHA-256"};
  }
  std::array<char, 64U * 1024U> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    if (count > 0 && EVP_DigestUpdate(
      context.get(), buffer.data(), static_cast<std::size_t>(count)) != 1)
    {
      throw std::runtime_error{"cannot update SHA-256"};
    }
  }
  if (!stream.eof()) {
    throw std::runtime_error{"cannot read complete file for SHA-256: " + file.string()};
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0U;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &length) != 1 || length != 32U) {
    throw std::runtime_error{"cannot finalize SHA-256"};
  }
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned int index = 0U; index < length; ++index) {
    result << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return result.str();
}

std::filesystem::path confinedPath(
  const std::filesystem::path & root, const std::filesystem::path & relative)
{
  if (relative.empty() || relative.is_absolute()) {
    throw std::invalid_argument{"bundle path must be nonempty and relative"};
  }
  for (const auto & component : relative) {
    if (component == "..") {
      throw std::invalid_argument{"bundle path traversal is forbidden"};
    }
  }
  const auto canonical_root = std::filesystem::weakly_canonical(root);
  const auto candidate = std::filesystem::weakly_canonical(canonical_root / relative);
  auto root_iterator = canonical_root.begin();
  auto candidate_iterator = candidate.begin();
  for (; root_iterator != canonical_root.end(); ++root_iterator, ++candidate_iterator) {
    if (candidate_iterator == candidate.end() || *root_iterator != *candidate_iterator) {
      throw std::invalid_argument{"bundle path escapes root"};
    }
  }
  return candidate;
}

}  // namespace a2w_fastlio_map
