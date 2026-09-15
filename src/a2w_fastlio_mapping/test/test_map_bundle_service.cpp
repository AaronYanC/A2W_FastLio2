#include <condition_variable>
#include <filesystem>
#include <future>
#include <mutex>

#include <gtest/gtest.h>

#include "a2w_fastlio_mapping/map_bundle_service.hpp"

namespace a2w_fastlio_mapping
{
namespace
{

TEST(MapBundleService, RejectsAbsoluteAndTraversalOutputPaths)
{
  const auto root = std::filesystem::temp_directory_path() / "a2w-service-root";
  MapBundleService service{root, 1U, [](const auto &, const auto &) {
      return a2w_fastlio_map::BundleWriteResult{};
    }};
  EXPECT_EQ(service.save("/absolute", {}).message, "output_path_not_confined");
  EXPECT_EQ(service.save("../escape", {}).message, "output_path_not_confined");
}

TEST(MapBundleService, ReportsBusyWhileSingleWorkerIsActive)
{
  const auto root = std::filesystem::temp_directory_path() / "a2w-service-busy";
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false;
  bool release = false;
  MapBundleService service{root, 1U, [&](const auto & path, const auto &) {
      {
        std::lock_guard<std::mutex> lock{mutex};
        entered = true;
      }
      condition.notify_all();
      std::unique_lock<std::mutex> lock{mutex};
      condition.wait(lock, [&]() {return release;});
      a2w_fastlio_map::BundleWriteResult result;
      result.success = true;
      result.resolved_path = path;
      return result;
    }};

  auto first = std::async(std::launch::async, [&]() {return service.save("first", {});});
  {
    std::unique_lock<std::mutex> lock{mutex};
    condition.wait(lock, [&]() {return entered;});
  }
  EXPECT_EQ(service.save("second", {}).message, "save_busy");
  {
    std::lock_guard<std::mutex> lock{mutex};
    release = true;
  }
  condition.notify_all();
  EXPECT_TRUE(first.get().success);
}

}  // namespace
}  // namespace a2w_fastlio_mapping
