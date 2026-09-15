#include "a2w_fastlio_localization/relocalization_worker.hpp"

#include <stdexcept>
#include <utility>

#include "a2w_fastlio_common/registration.hpp"

namespace a2w_fastlio_localization
{

RelocalizationWorker::RelocalizationWorker(
  EvaluateFunction evaluator, ResultCallback callback, const std::size_t queue_capacity)
: evaluator_{std::move(evaluator)}, callback_{std::move(callback)},
  queue_capacity_{queue_capacity}
{
  if (!evaluator_ || !callback_ || queue_capacity_ == 0U) {
    throw std::invalid_argument{"relocalization worker requires callbacks and bounded capacity"};
  }
  thread_ = std::thread{&RelocalizationWorker::run, this};
}

RelocalizationWorker::~RelocalizationWorker()
{
  {
    std::lock_guard<std::mutex> lock{mutex_};
    stopping_ = true;
    ++generation_;
    queue_.clear();
  }
  condition_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool RelocalizationWorker::enqueue(const a2w_fastlio_common::FrontendFrame & frame)
{
  if (frame.stamp_ns < 0 || !a2w_fastlio_common::isFinitePose(frame.odom_pose) ||
    !frame.body_cloud || frame.body_cloud->empty())
  {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (stopping_ || queue_.size() >= queue_capacity_) {
      return false;
    }
    queue_.push_back(Job{frame, generation_});
  }
  condition_.notify_one();
  return true;
}

void RelocalizationWorker::cancelPending()
{
  {
    std::lock_guard<std::mutex> lock{mutex_};
    ++generation_;
    queue_.clear();
  }
  condition_.notify_all();
}

std::size_t RelocalizationWorker::queued() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  return queue_.size();
}

void RelocalizationWorker::run()
{
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock{mutex_};
      condition_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
      if (stopping_) {
        return;
      }
      job = queue_.front();
      queue_.pop_front();
    }
    RelocalizationResult evaluation;
    try {
      evaluation = evaluator_(job.frame);
    } catch (const std::exception & error) {
      evaluation.reason = std::string{"worker_evaluation_failed:"} + error.what();
    } catch (...) {
      evaluation.reason = "worker_evaluation_failed:unknown";
    }
    {
      std::lock_guard<std::mutex> lock{mutex_};
      if (stopping_ || job.generation != generation_) {
        continue;
      }
    }
    try {
      callback_(RelocalizationWorkResult{job.frame, std::move(evaluation)});
    } catch (...) {
      // A callback failure must not terminate the worker thread.
    }
  }
}

}  // namespace a2w_fastlio_localization
