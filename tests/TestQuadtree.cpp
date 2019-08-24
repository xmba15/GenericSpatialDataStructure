/**
 * @file    TestQuadtree.cpp
 *
 * @brief   Test Basic Funtion of Quadtree
 *
 * @author  btran
 *
 * @date    2019-07-03
 *
 * Copyright (c) organization
 *
 */

#include "gtest/gtest.h"
#include <ctime>
#include <spatial_partioning/Quadtree.hpp>

class TestQuadtree : public ::testing::Test
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

TEST_F(TestQuadtree, TestInitialization)
{
    EXPECT_TRUE(true);
}
