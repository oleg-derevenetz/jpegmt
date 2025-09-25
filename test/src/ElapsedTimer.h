#pragma once

#include <cstdint>

class ElapsedTimer
{
public:
  ElapsedTimer();

  void start();
  uint64_t restart();
  uint64_t elapsed() const;

private:
  uint64_t m_startTime = 0;
};
