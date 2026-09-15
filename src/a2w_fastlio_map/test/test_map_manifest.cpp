#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "a2w_fastlio_map/map_io.hpp"
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
    const auto base = std::filesystem::temp_directory_path() / "a2w-map-manifest-XXXXXX";
    auto pattern = base.string();
    root_ = ::mkdtemp(pattern.data());
  }
  ~TemporaryDirectory() {std::filesystem::remove_all(root_);}
  const std::filesystem::path & path() const {return root_;}

private:
  std::filesystem::path root_;
};

void write(const std::filesystem::path & path, const std::string & value)
{
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream{path, std::ios::binary};
  stream << value;
}

TEST(MapIO, ComputesStableStreamingSha256)
{
  TemporaryDirectory directory;
  const auto file = directory.path() / "abc.bin";
  write(file, "abc");
  EXPECT_EQ(
    sha256File(file),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(MapIO, ConfinesRelativePathsAndRejectsSymlinkEscape)
{
  TemporaryDirectory directory;
  TemporaryDirectory outside;
  write(directory.path() / "safe/file.txt", "safe");
  write(outside.path() / "secret.txt", "secret");
  std::filesystem::create_symlink(outside.path() / "secret.txt", directory.path() / "escape");

  EXPECT_EQ(
    confinedPath(directory.path(), "safe/file.txt"),
    std::filesystem::weakly_canonical(directory.path() / "safe/file.txt"));
  EXPECT_THROW(confinedPath(directory.path(), "/absolute"), std::invalid_argument);
  EXPECT_THROW(confinedPath(directory.path(), "../escape"), std::invalid_argument);
  EXPECT_THROW(confinedPath(directory.path(), "escape"), std::invalid_argument);
}

TEST(MapManifest, IsDeterministicSupportsSpacesAndVerifiesChanges)
{
  TemporaryDirectory directory;
  write(directory.path() / "z file.bin", "z");
  write(directory.path() / "nested/a.bin", "a");
  const auto manifest = MapManifest::build(
    directory.path(), {"z file.bin", "nested/a.bin"});
  ASSERT_EQ(manifest.entries().size(), 2U);
  EXPECT_EQ(manifest.entries()[0].relative_path, std::filesystem::path{"nested/a.bin"});
  manifest.write(directory.path());

  const auto loaded = MapManifest::read(directory.path());
  EXPECT_TRUE(loaded.verify(directory.path()).valid);
  write(directory.path() / "z file.bin", "modified");
  const auto modified = loaded.verify(directory.path());
  EXPECT_FALSE(modified.valid);
  EXPECT_EQ(modified.errors.front(), "hash_mismatch:z file.bin");
}

TEST(MapManifest, DetectsMissingFilesAndRejectsDuplicateEntries)
{
  TemporaryDirectory directory;
  write(directory.path() / "one", "1");
  EXPECT_THROW(
    MapManifest::build(directory.path(), {"one", "one"}), std::invalid_argument);
  auto manifest = MapManifest::build(directory.path(), {"one"});
  std::filesystem::remove(directory.path() / "one");
  const auto verification = manifest.verify(directory.path());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.errors.front(), "missing:one");
}

TEST(MapManifest, ReaderRejectsMalformedAndDuplicateManifestLines)
{
  TemporaryDirectory directory;
  write(directory.path() / "manifest.sha256", "not-a-hash  file\n");
  EXPECT_THROW(MapManifest::read(directory.path()), std::runtime_error);
  const std::string hash(64U, 'a');
  write(
    directory.path() / "manifest.sha256",
    hash + "  file\n" + hash + "  file\n");
  EXPECT_THROW(MapManifest::read(directory.path()), std::runtime_error);
}

}  // namespace
}  // namespace a2w_fastlio_map
