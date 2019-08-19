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
#include <sstream>
#include <string>

namespace algo
{
uint64_t Part1By2(uint32_t n);
std::string bin(uint32_t n);
std::string bin(uint64_t n);

uint64_t Morton3(uint32_t x, uint32_t y, uint32_t z)
{
    return (Part1By2(z) << 2) | (Part1By2(y) << 1) | Part1By2(x);
}

uint64_t Part1By2(uint32_t n)
{
    uint64_t result = n & 0x1fffff;
    result = (result | result << 32) & 0x1f00000000ffff;
    result = (result | result << 16) & 0x1f0000ff0000ff;
    result = (result | result << 8) & 0x100f00f00f00f00f;
    result = (result | result << 4) & 0x10c30c30c30c30c3;
    result = (result | result << 2) & 0x1249249249249249;

    return result;
}

std::string bin(uint32_t n)
{
    std::stringstream ss;
    for (uint32_t i = 1 << 31; i > 0; i >>= 1) {
        (n & i) ? ss << 1 : ss << 0;
    }

    return ss.str();
}

std::string bin(uint64_t n)
{
    std::stringstream ss;
    for (uint64_t i = 1UL << 63; i > 0; i >>= 1) {
        (n & i) ? ss << 1 : ss << 0;
    }

    return ss.str();
}

}  // namespace algo
