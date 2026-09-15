#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "a2w_fastlio_map/map_bundle_writer.hpp"

namespace a2w_fastlio_mapping
{

using MapBundleWriterFunction = std::function<a2w_fastlio_map::BundleWriteResult(
    const std::filesystem::path &, const a2w_fastlio_map::MapBundleData &)>;

class MapBundleService
{
public:
  MapBundleService(
    std::filesystem::path bundle_root, std::size_t capacity,
    MapBundleWriterFunction writer = {});
  ~MapBundleService();

  MapBundleService(const MapBundleService &) = delete;
  MapBundleService & operator=(const MapBundleService &) = delete;

  a2w_fastlio_map::BundleWriteResult save(
    const std::string & relative_output,
    a2w_fastlio_map::MapBundleData data);

private:
  struct Job
  {
    std::filesystem::path output;
    a2w_fastlio_map::MapBundleData data;
    std::promise<a2w_fastlio_map::BundleWriteResult> result;
  };

  void run();

  std::filesystem::path bundle_root_;
  std::size_t capacity_;
  MapBundleWriterFunction writer_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::unique_ptr<Job>> queue_;
  bool active_{false};
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace a2w_fastlio_mapping
