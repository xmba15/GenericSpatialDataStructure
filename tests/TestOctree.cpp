/**
 * @file    TestOctree.cpp
 *
 * @brief   Test Basic Funtion of Octree
 *
 * @author  btran
 *
 * @date    2019-07-03
 *
 * Copyright (c) organization
 *
 */

#include "gtest/gtest.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <random>

#include <spatial_partioning/Octree.hpp>

namespace testing
{
template <typename DATA_TYPE>
void EXPECT_POINT_DOUBLE_EQ(const typename algo::Octree<DATA_TYPE>::PointType& p1,
                            const typename algo::Octree<DATA_TYPE>::PointType& p2)
{
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(p1[i], p2[i]);
    }
}

}  // namespace testing

class TestOctree : public ::testing::Test
{
 protected:
    using Octree = algo::Octree<double>;

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

TEST_F(TestOctree, TestInitialization)
{
    Octree::VecPointType points{{30, 57, 45}, {15, 6, -89},    {68, 70, 12.3}, {-1, 5, 2},        {10, 67, 399.2},
                                {45, 9, 33},  {100, 6, 212.2}, {20, 45, 23.2}, {-200, -200, -11}, {200, 200, 234}};
    Octree octree(points);

    {
        Octree::PointType query({68.2, 70, 12.1});
        uint32_t neighborIdx = octree.findNeighbor(query);
        Octree::PointType neighbor = points[neighborIdx];
        Octree::PointType expectedNeighbor{68, 70, 12.3};

        testing::EXPECT_POINT_DOUBLE_EQ<double>(neighbor, expectedNeighbor);
    }
}

TEST_F(TestOctree, TestKNN)
{
    const double LOWER_BOUND = -500;
    const double UPPER_BOUND = 500;
    const int NUM_POINTS = 400;
    const int NUM_POINTS_TO_CHECK = 100;

    Octree::VecPointType points;
    Octree::VecPointType pointsToCheck;
    points.reserve(NUM_POINTS);
    pointsToCheck.reserve(NUM_POINTS_TO_CHECK);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine re(seed);

    std::uniform_real_distribution<double> unif(LOWER_BOUND, UPPER_BOUND);

    for (size_t i = 0; i < NUM_POINTS; ++i) {
        double x = unif(re);
        double y = unif(re);
        double z = unif(re);
        points.emplace_back(Octree::PointType{x, y, z});
    }

    for (size_t i = 0; i < NUM_POINTS_TO_CHECK; ++i) {
        double x = unif(re);
        double y = unif(re);
        double z = unif(re);
        pointsToCheck.emplace_back(Octree::PointType{x, y, z});
    }

    const Octree octree(points);

    {
        for (const Octree::PointType& pointToCheck : pointsToCheck) {
            auto tempPV(points);

            std::sort(tempPV.begin(), tempPV.end(),
                      [pointToCheck](const Octree::PointType& p1, const Octree::PointType& p2) {
                          return (p1 - pointToCheck).norm() < (p2 - pointToCheck).norm();
                      });

            const uint32_t neighborIdx = octree.findNeighbor(pointToCheck);
            const Octree::PointType& neighbor = points[neighborIdx];
            const Octree::PointType& expectedNeighbor = tempPV.front();

            testing::EXPECT_POINT_DOUBLE_EQ<double>(neighbor, expectedNeighbor);
        }
    }

    {
        const int Ks[] = {1, 3, 12, 300, 450};

        for (const Octree::PointType& pointToCheck : pointsToCheck) {
            auto tempPV(points);

            std::sort(tempPV.begin(), tempPV.end(),
                      [pointToCheck](const Octree::PointType& p1, const Octree::PointType& p2) {
                          return (p1 - pointToCheck).norm() < (p2 - pointToCheck).norm();
                      });

            for (const int K : Ks) {
                const int kAlpha = K > NUM_POINTS ? NUM_POINTS : K;

                const Octree::VecPointType expectedKnnPoints(tempPV.begin(), tempPV.begin() + kAlpha);

                std::vector<uint32_t> knnIndices = octree.knn(pointToCheck, K);
                ASSERT_EQ(knnIndices.size(), kAlpha);

                Octree::VecPointType knnPoints;
                knnPoints.reserve(kAlpha);

                std::transform(knnIndices.begin(), knnIndices.end(), std::back_inserter(knnPoints),
                               [&points](const uint32_t idx) { return points[idx]; });

                ASSERT_EQ(expectedKnnPoints.size(), knnPoints.size());
                auto expectedIt = expectedKnnPoints.cbegin();
                auto it = knnPoints.cbegin();

                for (; expectedIt != expectedKnnPoints.cend(); ++expectedIt, ++it) {
                    testing::EXPECT_POINT_DOUBLE_EQ<double>(*it, *expectedIt);
                }
            }
        }
    }
}
