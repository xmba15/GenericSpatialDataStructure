/**
 * @file    ExSpaceFillingCurve.cpp
 *
 * @brief   Example for Space Filling Curve
 *
 * @author  btran
 *
 * @date    2019-07-09
 *
 * Copyright (c) organization
 *
 */

#include <iostream>
#include <spatial_partioning/SpaceFillingCurve.hpp>
#include <spatial_partioning/BitTwiddling.hpp>

int main(int argc, char* argv[])
{
    uint32_t a = 2147483647;
    // auto mortonCode = algo::Morton3(a, a + 1, a + 2);
    // std::cout << algo::bin(mortonCode) << "\n";
    // std::cout << mortonCode << "\n";

    std::cout << algo::bin(a) << "\n";
    std::cout << algo::bin(algo::Part1By1(a)) << "\n";

    uint32_t x = 10;
    uint32_t y = 22;
    uint32_t z = 23;

    auto mCode = algo::Morton3(x, y, z);
    std::cout << mCode << "\n";

    auto childCode = (mCode << 3) + 5;
    std::cout << childCode << "\n";

    std::cout << algo::bin(mCode) << "\n";
    std::cout << algo::bin(childCode) << "\n";

    return 0;
}
