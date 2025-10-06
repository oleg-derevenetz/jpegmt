#pragma once 

#include <cstdint>

#ifndef FORCE_INLINE
#ifdef PLATFORM_COMPILER_MSVC
#define FORCE_INLINE __forceinline
#elif defined(PLATFORM_COMPILER_GNU)
#define FORCE_INLINE __attribute__((always_inline)) inline
#else
#define FORCE_INLINE inline
#endif
#endif

namespace
{

struct NoReturnValueCallable
{
  typedef void ReturnType;
  static void makeDefaultValue() {}
};

template<typename Callable>
struct SimdFunctionChooser
{
  template <typename T, typename... Args>
  static typename Callable::ReturnType perform(int simdLength, Args&& ...args);
};

template<typename Callable, typename T, typename... Args> struct CallHelper;

template<typename Callable, typename... Args>
struct CallHelper<Callable, int8_t, Args...>
{
  static typename Callable::ReturnType perform(int simdLength, Args ...args)
  {
    switch (simdLength)
    {
    case 32:
      return Callable::template perform<int8_t, 32>(args...);
    case 16:
      return Callable::template perform<int8_t, 16>(args...);
    case 1:
      return Callable::template perform<int8_t, 1>(args...);
    }

    return Callable::makeDefaultValue();
  }
};

template<typename Callable, typename... Args>
struct CallHelper<Callable, int16_t, Args...>
{
  static typename Callable::ReturnType perform(int simdLength, Args ...args)
  {
    switch (simdLength)
    {
    case 16:
      return Callable::template perform<int16_t, 16>(args...);
    case 8:
      return Callable::template perform<int16_t, 8>(args...);
    case 1:
      return Callable::template perform<int16_t, 1>(args...);
    }

    return Callable::makeDefaultValue();
  }
};

template<typename Callable, typename... Args>
struct CallHelper<Callable, int32_t, Args...>
{
  static typename Callable::ReturnType perform(int simdLength, Args ...args)
  {
    switch (simdLength)
    {
    case 8:
      return Callable::template perform<int32_t, 8>(args...);
    case 4:
      return Callable::template perform<int32_t, 4>(args...);
    case 1:
      return Callable::template perform<int32_t, 1>(args...);
    }

    return Callable::makeDefaultValue();
  }
};

template<typename Callable>
template<typename T, typename ...Args>
inline typename Callable::ReturnType SimdFunctionChooser<Callable>::perform(int simdLength, Args&& ...args)
{
  return CallHelper<Callable, T, Args...>::perform(simdLength, args...);
}

}
