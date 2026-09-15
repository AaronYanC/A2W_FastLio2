#include "a2w_fastlio_mapping/map_bundle_service.hpp"

#include <stdexcept>
#include <utility>

#include "a2w_fastlio_map/map_io.hpp"

namespace a2w_fastlio_mapping
{

MapBundleService::MapBundleService(
  std::filesystem::path bundle_root, const std::size_t capacity,
  MapBundleWriterFunction writer)
: bundle_root_{std::filesystem::absolute(std::move(bundle_root))},
  capacity_{capacity}, writer_{std::move(writer)}
{
  if (capacity_ == 0U || bundle_root_.empty()) {
    throw std::invalid_argument{"invalid Map Bundle service configuration"};
  }
  std::filesystem::create_directories(bundle_root_);
  if (!writer_) {
    writer_ = [](const auto & output, const auto & data) {
        return a2w_fastlio_map::MapBundleWriter{}.write(output, data);
      };
  }
  worker_ = std::thread{&MapBundleService::run, this};
}

MapBundleService::~MapBundleService()
{
  {
    std::lock_guard<std::mutex> lock{mutex_};
    stopping_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

a2w_fastlio_map::BundleWriteResult MapBundleService::save(
  const std::string & relative_output, a2w_fastlio_map::MapBundleData data)
{
  std::filesystem::path output;
  try {
    output = a2w_fastlio_map::confinedPath(bundle_root_, relative_output);
  } catch (const std::exception &) {
    return {false, "output_path_not_confined", data.metadata.bundle_uuid,
      data.keyframes.size(), {}};
  }
  auto job = std::make_unique<Job>();
  job->output = std::move(output);
  job->data = std::move(data);
  auto future = job->result.get_future();
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (stopping_) {
      return {false, "service_stopping", job->data.metadata.bundle_uuid,
        job->data.keyframes.size(), {}};
    }
    if ((active_ ? 1U : 0U) + queue_.size() >= capacity_) {
      return {false, "save_busy", job->data.metadata.bundle_uuid,
        job->data.keyframes.size(), {}};
    }
    queue_.push_back(std::move(job));
  }
  condition_.notify_one();
  return future.get();
}

void MapBundleService::run()
{
  while (true) {
    std::unique_ptr<Job> job;
    {
      std::unique_lock<std::mutex> lock{mutex_};
      condition_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
      if (stopping_ && queue_.empty()) {
        return;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
      active_ = true;
    }
    a2w_fastlio_map::BundleWriteResult result;
    try {
      result = writer_(job->output, job->data);
    } catch (const std::exception & error) {
      result = {false, error.what(), job->data.metadata.bundle_uuid,
        job->data.keyframes.size(), {}};
    }
    job->result.set_value(std::move(result));
    {
      std::lock_guard<std::mutex> lock{mutex_};
      active_ = false;
    }
  }
}

}  // namespace a2w_fastlio_mapping
