/**
 * @file    SpaceFillingCurve.hpp
 *
 * @brief   Implementation of Popular Spacial Filling Curves
 *
 * @author  bt
 *
 * @date    2019-07-09
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cstdint>
#include <functional>
#include <sstream>
#include <string>

namespace algo
{
inline uint64_t Part1By2(const uint32_t n)
{
    uint64_t result = n & 0x1fffff;
    result = (result | result << 32) & 0x1f00000000ffff;
    result = (result | result << 16) & 0x1f0000ff0000ff;
    result = (result | result << 8) & 0x100f00f00f00f00f;
    result = (result | result << 4) & 0x10c30c30c30c30c3;
    result = (result | result << 2) & 0x1249249249249249;

    return result;
}

inline uint64_t Part1By1(const uint32_t n)
{
    uint64_t result = n;
    result = (result | result << 16) & 0xffff0000ffff;
    result = (result | result << 8) & 0xff00ff00ff00ff;
    result = (result | result << 4) & 0xf0f0f0f0f0f0f0f;
    result = (result | result << 2) & 0x3333333333333333;
    result = (result | result << 1) & 0x5555555555555555;

    return result;
}

inline uint64_t Morton3(uint32_t x, uint32_t y, uint32_t z)
{
    return (Part1By2(z) << 2) | (Part1By2(y) << 1) | Part1By2(x);
}

inline uint64_t Morton3(uint32_t x, uint32_t y)
{
    return (Part1By1(y) << 1) | Part1By1(x);
}

inline uint64_t computeXYZKey(uint64_t x, uint64_t y, uint64_t z, int8_t maxLevel)
{
    uint64_t key = x | (y << (maxLevel + 1)) | (z << 2 * (maxLevel + 1));

    return key;
}

inline uint64_t computeXYZKey(uint64_t x, uint64_t y, int8_t maxLevel)
{
    uint64_t key = x | (y << (maxLevel + 1));

    return key;
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

}  // namespace algo
