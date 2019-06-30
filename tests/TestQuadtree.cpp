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

#include <chrono>
#include <ctime>
#include <iomanip>
#include <random>

#include <spatial_partioning/Quadtree.hpp>

namespace testing
{
template <typename DATA_TYPE>
void EXPECT_POINT_DOUBLE_EQ(const typename algo::Quadtree<DATA_TYPE>::PointType& p1,
                            const typename algo::Quadtree<DATA_TYPE>::PointType& p2)
{
    for (size_t i = 0; i < 2; ++i) {
        EXPECT_DOUBLE_EQ(p1[i], p2[i]);
    }
}

}  // namespace testing

class TestQuadtree : public ::testing::Test
{
 protected:
    using Quadtree = algo::Quadtree<double>;

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
    Quadtree::VecPointType points{{30, 57}, {15, 6},  {68, 70}, {-1, 5},      {10, 67},
                                  {45, 9},  {100, 6}, {20, 45}, {-200, -200}, {200, 200}};
    Quadtree quadtree(points);

    {
        Quadtree::PointType query({58.2, 70.2});
        uint32_t neighborIdx = quadtree.findNeighbor(query);
        Quadtree::PointType neighbor = points[neighborIdx];
        Quadtree::PointType expectedNeighbor{68, 70};

        testing::EXPECT_POINT_DOUBLE_EQ<double>(neighbor, expectedNeighbor);
    }
}

TEST_F(TestQuadtree, TestKNN)
{
    const double LOWER_BOUND = -500;
    const double UPPER_BOUND = 500;
    const int NUM_POINTS = 400;
    const int NUM_POINTS_TO_CHECK = 100;

    Quadtree::VecPointType points;
    Quadtree::VecPointType pointsToCheck;
    points.reserve(NUM_POINTS);
    pointsToCheck.reserve(NUM_POINTS_TO_CHECK);

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine re(seed);

    std::uniform_real_distribution<double> unif(LOWER_BOUND, UPPER_BOUND);

    for (size_t i = 0; i < NUM_POINTS; ++i) {
        double x = unif(re);
        double y = unif(re);
        points.emplace_back(Quadtree::PointType{x, y});
    }

    for (size_t i = 0; i < NUM_POINTS_TO_CHECK; ++i) {
        double x = unif(re);
        double y = unif(re);
        pointsToCheck.emplace_back(Quadtree::PointType{x, y});
    }

    const Quadtree quadtree(points);

    {
        for (const Quadtree::PointType& pointToCheck : pointsToCheck) {
            auto tempPV(points);

            std::sort(tempPV.begin(), tempPV.end(),
                      [pointToCheck](const Quadtree::PointType& p1, const Quadtree::PointType& p2) {
                          return (p1 - pointToCheck).norm() < (p2 - pointToCheck).norm();
                      });

            const uint32_t neighborIdx = quadtree.findNeighbor(pointToCheck);
            const Quadtree::PointType& neighbor = points[neighborIdx];
            const Quadtree::PointType& expectedNeighbor = tempPV.front();

            testing::EXPECT_POINT_DOUBLE_EQ<double>(neighbor, expectedNeighbor);
        }
    }

    {
        const int Ks[] = {1, 3, 12, 300, 450};

        for (const Quadtree::PointType& pointToCheck : pointsToCheck) {
            auto tempPV(points);

            std::sort(tempPV.begin(), tempPV.end(),
                      [pointToCheck](const Quadtree::PointType& p1, const Quadtree::PointType& p2) {
                          return (p1 - pointToCheck).norm() < (p2 - pointToCheck).norm();
                      });

            for (const int K : Ks) {
                const int kAlpha = K > NUM_POINTS ? NUM_POINTS : K;

                const Quadtree::VecPointType expectedKnnPoints(tempPV.begin(), tempPV.begin() + kAlpha);

                std::vector<uint32_t> knnIndices = quadtree.knn(pointToCheck, K);
                ASSERT_EQ(knnIndices.size(), kAlpha);

                Quadtree::VecPointType knnPoints;
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
