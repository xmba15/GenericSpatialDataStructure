/**
 * @file    ExQuadtree.cpp
 *
 * @brief   Example for Quadtree
 *
 * @author  btran
 *
 * @date    2019-07-01
 *
 * Copyright (c) organization
 *
 */

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <spatial_indexing/Quadtree.hpp>

int main(int argc, char* argv[])
{
    using Quadtree = algo::Quadtree<double>;

    {
        Quadtree::VecPointType points{{30, 57}, {15, 6},  {68, 70}, {-1, 5},      {10, 67},
                                      {45, 9},  {100, 6}, {20, 45}, {-200, -200}, {200, 200}};

        Quadtree quadtree(points);

        Quadtree::PointType query({-20, -34.3});

        bool result = quadtree.insideQuadtreeSpace(query);

        std::cout << quadtree.traversal().str() << "\n";

        {
            std::cout << "test computation time between naive nearest search && 1nn by quadtree: \n";
            const double LOWER_BOUND = -500;
            const double UPPER_BOUND = 500;
            const int NUM_POINTS = 400;
            const int NUM_POINTS_TO_CHECK = 10;
            const int K = 1;

            Quadtree::VecPointType points;
            Quadtree::VecPointType pointsToCheck;
            points.reserve(NUM_POINTS);
            pointsToCheck.reserve(NUM_POINTS_TO_CHECK);

            std::uniform_real_distribution<double> unif(LOWER_BOUND, UPPER_BOUND);
            std::default_random_engine re;

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

            {
                std::cout << "naive method: \n";
                auto start = std::chrono::high_resolution_clock::now();
                for (const Quadtree::PointType& pointToCheck : pointsToCheck) {
                    auto tempPV(points);
                    std::sort(tempPV.begin(), tempPV.end(),
                              [pointToCheck](const Quadtree::PointType& p1, const Quadtree::PointType& p2) {
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
                std::cout << "1nn by quadtree: \n";
                auto start = std::chrono::high_resolution_clock::now();
                Quadtree quadtree(points);

                for (const Quadtree::PointType& pointToCheck : pointsToCheck) {
                    quadtree.findNeighbor(pointToCheck);
                }

                auto end = std::chrono::high_resolution_clock::now();
                double executedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
                executedTime *= 1e-9;
                std::cout << "Time taken by program is : " << std::fixed << executedTime << std::setprecision(6);
                std::cout << " sec\n";
            }
        }
    }

    return 0;
}
