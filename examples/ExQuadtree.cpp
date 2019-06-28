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

#include <iostream>
#include <spatial_data_structure/Quadtree.hpp>

int main(int argc, char* argv[])
{
    using Quadtree = algo::Quadtree<double>;
    Quadtree::BoundingBox bbox(0, 0, 100, 100);

    Quadtree::Ptr quadTree = std::make_shared<Quadtree>(bbox);
    Quadtree::VecPointType pV{{30, 57}, {15, 6}, {68, 70}, {-1, 5}, {10, 67}, {45, 9}, {100, 6}, {20, 45}};

    quadTree->insertElems(pV);

    Quadtree::PointType pointToCheck{10, 20};
    std::cout << "distance: \n";
    for (auto& elem : quadTree->queryRange(bbox)) {
        std::cout << "-------------\n";
        std::cout << elem << "\n";
        std::cout << "distance: " << (elem - pointToCheck).norm() << "\n";
    }
    std::cout << "end of calculating distance: \n";

    Quadtree::VecPointType knn = quadTree->nearestSearch(Quadtree::PointType{10, 20}, 4);
    std::cout << "k nearest search\n";
    for (auto& elem : knn) {
        std::cout << elem << "\n";
        std::cout << "---\n";
    }

    std::cout << "end of k nearest search \n";

    quadTree->deleteElems(pV);

    Quadtree::VecPointType v = quadTree->queryRange(bbox);

    for (auto& elem : v) {
        std::cout << elem << "\n";
        std::cout << "---"
                  << "\n";
    }

    Quadtree::Ptr quadrantPtr = quadTree->searchElem(Quadtree::PointType{15, 6});

    if (quadrantPtr) {
        std::cout << "found the element"
                  << "\n";
        std::cout << quadrantPtr->bbox() << "\n";
    } else {
        std::cout << "nothing found"
                  << "\n";
    }

    return 0;
}
