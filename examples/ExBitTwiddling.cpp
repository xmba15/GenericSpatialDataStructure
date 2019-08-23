/**
 * @file    ExBitTwiddling.cpp
 *
 * @author  btran
 *
 * @date    2019-08-20
 *
 * Copyright (c) organization
 *
 */

#include <iostream>

#include <spatial_partioning/BitTwiddling.hpp>

int main(int argc, char* argv[])
{
    double tmp = 1.14;

    auto expo = algo::ieeeExponent(tmp);
    auto mantissa = algo::ieeeMantissa(tmp);

    std::cout << expo << "\n";
    std::cout << mantissa << "\n";

    int target = 3;
    std::cout << algo::bin(uint32_t(target)) << "\n";
    std::cout << algo::lzcount32(uint32_t(target)) << "\n";
    std::cout << __builtin_clz(target) << "\n";

    float a = 19.565;
    float b = 19.5;

    std::cout << algo::xormsb(a, b) << "\n";

    return 0;
}
