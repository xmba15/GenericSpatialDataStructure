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

namespace algo
{
#define UINT64_LITERAL(x) static_cast<uint64_t>(x##ull)
#define INT64_LITERAL(x) static_cast<int64_t>(x##ll)

#define FLOAT_SIGNMASK (1 << 31);
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

inline uint32_t ieee_exponent(float f)
{
    union32_t u;
    u.f = f;
    return (u.i & 0x7f800000) >> 23;
}

inline uint32_t ieee_mantissa(float f)
{
    union32_t u;
    u.f = f;
    return (u.i & 0x007FFFFF);
}

inline uint64_t ieee_exponent(double f)
{
    union64_t u;
    u.f = f;
    return (u.i & INT64_LITERAL(0x7ff0000000000000)) >> 52;
}

inline uint64_t ieee_mantissa(double f)
{
    union64_t u;
    u.f = f;
    return (u.i & INT64_LITERAL(0x000fffffffffffff));
}
//@}

}  // namespace algo
