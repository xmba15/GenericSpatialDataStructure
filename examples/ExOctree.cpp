/**
 * @file    ExOctree.cpp
 *
 * @brief   Example for Octree
 *
 * @author  btran
 *
 * @date    2019-07-02
 *
 * Copyright (c) organization
 *
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <spatial_partioning/Octree.hpp>

int main(int argc, char* argv[])
{
    using Octree = algo::Octree<double>;

    {
        Octree::VecPointType points{{30.0, 57, 15}, {15, 6, 78},   {68, 70, 43}, {-1, 5, -4},
                                    {10, 67, 30},   {45, 9, 16.7}, {100, 6, 0},  {20, 45, 65.3},
                                    {45, 67, 89},   {12, 34, 56},  {67, 12, 34}};

        Octree octree(points);

        Octree::PointType query({12.2, 34.5, 56.8});

        bool result = octree.insideSpace(query);

        int nnIdx = octree.findNeighbor(query);
        std::cout << points[nnIdx] << "\n";

        std::cout << octree.traversal().str() << "\n";

        {
            std::cout << "test computation time between naive nearest search && knn by octree: \n";
            const double LOWER_BOUND = 0;
            const double UPPER_BOUND = 200;
            const int NUM_POINTS = 5000;
            const int NUM_POINTS_TO_CHECK = 20;
            const int K = 1;

            Octree::VecPointType points;
            Octree::VecPointType pointsToCheck;
            points.reserve(NUM_POINTS);
            pointsToCheck.reserve(NUM_POINTS_TO_CHECK);

            std::uniform_real_distribution<double> unif(LOWER_BOUND, UPPER_BOUND);
            std::default_random_engine re;

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

            {
                std::cout << "naive method: \n";
                auto start = std::chrono::high_resolution_clock::now();
                for (const Octree::PointType& pointToCheck : pointsToCheck) {
                    auto tempPV(points);
                    std::sort(tempPV.begin(), tempPV.end(),
                              [pointToCheck](const Octree::PointType& p1, const Octree::PointType& p2) {
                                  return (p1 - pointToCheck).norm() < (p2 - pointToCheck).norm();
                              });
                }

                auto end = std::chrono::high_resolution_clock::now();
                double executedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

                executedTime *= 1e-9;
                std::cout << "Time taken by program is : " << std::fixed << executedTime << std::setprecision(6);
                std::cout << " sec\n";
            }

            {
                std::cout << "knn by octree: \n";
                auto start = std::chrono::high_resolution_clock::now();
                Octree octree(points);

                for (const Octree::PointType& pointToCheck : pointsToCheck) {
                    octree.findNeighbor(pointToCheck);
                }

                auto end = std::chrono::high_resolution_clock::now();
                double executedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
                executedTime *= 1e-9;
                std::cout << "Time taken by program is : " << std::fixed << executedTime << std::setprecision(6);
                std::cout << " sec\n";
            }

            {
                Octree octree(points);

                for (const Octree::PointType& pointToCheck : pointsToCheck) {
                    auto tempPV(points);
                    std::sort(tempPV.begin(), tempPV.end(),
                              [pointToCheck](const Octree::PointType& p1, const Octree::PointType& p2) {
                                  return (p1 - pointToCheck).norm() < (p2 - pointToCheck).norm();
                              });

                    int nearestNeighborIdx = octree.findNeighbor(pointToCheck);
                    Octree::PointType nn = points[nearestNeighborIdx];

                    std::cout << (nn - tempPV.front()) << "\n";
                }
            }
        }
    }

    return 0;
}
