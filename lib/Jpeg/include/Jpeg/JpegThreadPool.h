#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace Jpeg
{

class ThreadPool
{
public:
  typedef std::function<void(int, int64_t, int64_t)> WorkerFunction;

  virtual ~ThreadPool() = default;

  int computeThreadCount(int64_t workItemCount) const;
  bool executeParallel(const WorkerFunction& f, int64_t workItemCount);
  virtual int getMaxThreadCount() const = 0;

protected:
  virtual void executeWorkers(const WorkerFunction& f, const std::vector< std::pair<int64_t, int64_t> >& workerRanges) = 0;
};

}
