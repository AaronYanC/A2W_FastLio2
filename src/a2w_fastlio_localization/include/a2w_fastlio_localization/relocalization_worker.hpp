#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

#include "a2w_fastlio_common/types.hpp"
#include "a2w_fastlio_localization/global_relocalizer.hpp"

namespace a2w_fastlio_localization
{

struct RelocalizationWorkResult
{
  a2w_fastlio_common::FrontendFrame frame{};
  RelocalizationResult result{};
};

class RelocalizationWorker
{
public:
  using EvaluateFunction = std::function<RelocalizationResult(
      const a2w_fastlio_common::FrontendFrame &)>;
  using ResultCallback = std::function<void(const RelocalizationWorkResult &)>;

  RelocalizationWorker(
    EvaluateFunction evaluator, ResultCallback callback, std::size_t queue_capacity);
  ~RelocalizationWorker();

  RelocalizationWorker(const RelocalizationWorker &) = delete;
  RelocalizationWorker & operator=(const RelocalizationWorker &) = delete;

  bool enqueue(const a2w_fastlio_common::FrontendFrame & frame);
  void cancelPending();
  std::size_t queued() const;

private:
  struct Job
  {
    a2w_fastlio_common::FrontendFrame frame;
    std::uint64_t generation;
  };

  void run();

  EvaluateFunction evaluator_;
  ResultCallback callback_;
  std::size_t queue_capacity_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Job> queue_;
  std::uint64_t generation_{0U};
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace a2w_fastlio_localization
