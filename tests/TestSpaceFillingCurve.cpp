/**
 * @file    TestSpaceFillingCurve.cpp
 *
 * @brief   Test Basic Method for Space Filling Curve
 *
 * @author  btran
 *
 * @date    2019-07-09
 *
 * Copyright (c) organization
 *
 */

#include "gtest/gtest.h"
#include <ctime>
#include <spatial_partioning/BitTwiddling.hpp>
#include <spatial_partioning/SpaceFillingCurve.hpp>

class TestSpaceFillingCurve : public ::testing::Test
{
 protected:
    void SetUp() override
    {
        start_time_ = time(nullptr);
    }

    void TearDown() override
    {
        const time_t end_time = time(nullptr);

        // expect test time less than 10 sec
        EXPECT_LE(end_time - start_time_, 10);
    }

    time_t start_time_;
};

TEST_F(TestSpaceFillingCurve, TestBinaryRepresentation)
{
    {
        uint32_t n = 1785;
        // std::string binaryForm32bit = robotics::bin(n);
        std::string expectedBinaryForm32bit = "00000000000000000000011011111001";

        EXPECT_EQ(binaryForm32bit, expectedBinaryForm32bit);
    }

    {
        uint64_t n = 124544;
        // std::string binaryForm64bit = robotics::bin(n);
        std::string expectedBinaryForm64bit = "0000000000000000000000000000000000000000000000011110011010000000";

        EXPECT_EQ(binaryForm64bit, expectedBinaryForm64bit);
    }
}
