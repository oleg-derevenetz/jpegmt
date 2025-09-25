#include "ElapsedTimer.h"

#include <Helper/Platform/os.h>

#ifdef PLATFORM_OS_WINDOWS
#include <windows.h>
#elif defined(PLATFORM_OS_POSIX)
#include <time.h>
#else
#error "not implemented"
#endif

#ifdef PLATFORM_OS_WINDOWS
static double initTimerResolution()
{
  LARGE_INTEGER frequency;

  QueryPerformanceFrequency(&frequency);
  return 1000000000.0/frequency.QuadPart;
}

static double timerResolution = initTimerResolution();
#endif

static uint64_t getCurrentTime()
{
#ifdef PLATFORM_OS_WINDOWS
  LARGE_INTEGER counts;

  QueryPerformanceCounter(&counts);
  return (uint64_t)(timerResolution * counts.QuadPart);
#elif defined(PLATFORM_OS_POSIX)
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return ts.tv_sec * 1000000000ull + ts.tv_nsec;
#else
#error "not implemented"
#endif
}

ElapsedTimer::ElapsedTimer() : m_startTime(getCurrentTime())
{
}

void ElapsedTimer::start()
{
  m_startTime = getCurrentTime();
}

uint64_t ElapsedTimer::restart()
{
  uint64_t startTime = m_startTime;
  m_startTime = getCurrentTime();
  return m_startTime - startTime;
}

uint64_t ElapsedTimer::elapsed() const
{
  return getCurrentTime() - m_startTime;
}
