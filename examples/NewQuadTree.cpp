#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <spatial_partioning/NewQuadtree.hpp>

int main(int argc, char* argv[])
{
    using Quadtree = algo2::Quadtree<double>;
    Quadtree::VecPointType points{{30, 57}, {15, 6},  {68, 70}, {-1, 5},      {10, 67},
                                  {45, 9},  {100, 6}, {20, 45}, {-200, -200}, {200, 200}};

    Quadtree quadtree(points);

    Quadtree::PointType query({-20, -34.3});

    bool result = quadtree.insideQuadtreeSpace(query);

    std::cout << quadtree.traversal().str() << "\n";

    return 0;
}
