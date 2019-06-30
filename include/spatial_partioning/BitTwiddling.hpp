/**
 * @file    BitTwiddling.hpp
 *
 * @author  btran
 *
 * @date    2019-08-20
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cstdint>
#include <sstream>
#include <string>

// [good source](http://graphics.stanford.edu/~seander/bithacks.html)

namespace algo
{
#define UINT64_LITERAL(x) static_cast<uint64_t>(x##ull)
#define INT64_LITERAL(x) static_cast<int64_t>(x##ll)

#define DOUBLE_SIGNMASK INT64_LITERAL(0x8000000000000000)

//@{
/** ieee format for single and format floating points.
 *  single format: sign: 1, exponent: 8, float: 23 bits (total 32 bits)
 *  double format: sign: 1, exponent: 11, float: 52 bits (total 64 bits)
 */
typedef union {
    float f;
    int32_t i;
    uint32_t u;
} union32_t;

typedef union {
    double f;
    int64_t i;
    uint64_t u;
} union64_t;

inline uint32_t ieeeExponent(const float f)
{
    union32_t u;
    u.f = f;
    return (u.i & 0x7f800000) >> 23;
}

inline uint32_t ieeeMantissa(const float f)
{
    union32_t u;
    u.f = f;
    return (u.i & 0x007FFFFF);
}

inline uint64_t ieeeExponent(const double f)
{
    union64_t u;
    u.f = f;
    return (u.i & INT64_LITERAL(0x7ff0000000000000)) >> 52;
}

inline uint64_t ieeeMantissa(const double f)
{
    union64_t u;
    u.f = f;
    return (u.i & INT64_LITERAL(0x000fffffffffffff));
}
//@}

/**
 *  @brief count leading zeros
 */
inline int lzcount32(uint32_t x)
{
    if (x == 0) {
        return 32;
    }

#if defined(__GNUC__)
    return __builtin_clz(x);
#else
    int r = 0;
    while (x) {
        r++;
        x >>= 1;
    }
    return (32 - r);
#endif
}

/**
 *  @brief count leading zeros
 */
inline int lzcount64(uint64_t x)
{
    if (x == 0) {
        return 64;
    }

#if defined(__GNUC__)
    if (sizeof(uint64_t) == sizeof(long))
        return __builtin_clzl(x);
    else
        return __builtin_clzll(x);
#else
    int r = 0;
    while (x) {
        r++;
        x >= 1;
    }
    return (64 - r);
#endif
}

inline std::string bin(const uint32_t n)
{
    std::stringstream ss;
    for (uint32_t i = 1 << 31; i > 0; i >>= 1) {
        (n & i) ? (ss << 1) : (ss << 0);
    }

    return ss.str();
}

inline std::string bin(const uint64_t n)
{
    std::stringstream ss;
    for (uint64_t i = 1UL << 63; i > 0; i >>= 1) {
        (n & i) ? (ss << 1) : (ss << 0);
    }

    return ss.str();
}

/**
 *  @brief return the most significant differing bit
 */
inline int msdb(const uint32_t a, const uint32_t b)
{
    return 32 - lzcount32(a ^ b);
}

/**
 *  @brief return the most significant differing bit
 */
int msdb(const uint64_t a, const uint64_t b)
{
    return 64 - lzcount64(a ^ b);
}

/**
 *  @brief determine the most significant differing bit of floating point type
 *
 */
template <typename FloatType> inline int xormsb(const FloatType a, const FloatType b)
{
    int x = ieeeExponent(a);
    int y = ieeeExponent(b);

    if (x == y) {
        int z = msdb(ieeeMantissa(a), ieeeMantissa(b));
        return x - z;
    }

    return (y < x) ? x : y;
}

}  // namespace algo
