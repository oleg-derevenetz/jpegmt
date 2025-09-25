#include <Jpeg/JpegThreadPool.h>

namespace Jpeg
{

int ThreadPool::computeThreadCount(int64_t workItemCount) const
{
  if (workItemCount <= 0)
    return 0;

  int nThreads = getMaxThreadCount();
  if (nThreads < 1)
    nThreads = 1;
  if (nThreads > workItemCount)
    nThreads = (int)workItemCount;

  return nThreads;
}

void ThreadPool::executeParallel(const WorkerFunction& f, int64_t workItemCount)
{
  int nThreads = computeThreadCount(workItemCount);
  int64_t itemsPerThread = workItemCount / nThreads;
  int64_t itemsLeft = workItemCount % nThreads;
  std::vector< std::pair<int64_t, int64_t> > ranges(nThreads);

  int64_t nextItem = 0;
  for(int i = 0; i < nThreads; i++)
  {
    int64_t firstItem = nextItem;
    nextItem += itemsPerThread + (i < itemsLeft ? 1 : 0);

    ranges[i] = std::make_pair(firstItem, nextItem - 1);
  }

  executeWorkers(f, ranges);
}

}
