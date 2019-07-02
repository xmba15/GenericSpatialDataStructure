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

#include <iostream>
#include <spatial_data_structure/Octree.hpp>

int main(int argc, char *argv[])
{
    using Octree = algo::Octree<double>;
    Octree::BoundingBox3D bbox(0, 0, 0, 100, 100, 100);
    Octree::Ptr octree = std::make_shared<Octree>(bbox);
    Octree::VecPointType pV{{30, 57, 15}, {15, 6, 78},   {68, 70, 43}, {-1, 5, -4},
                            {10, 67, 30}, {45, 9, 16.7}, {100, 6, 0},  {20, 45, 65.3}};

    octree->insertElems(pV);

    std::cout << "Number of points: " << octree->pointSize() << "\n";

    Octree::VecPointType v = octree->queryRange(bbox);

    for (auto &elem : v) {
        std::cout << elem << "\n";
        std::cout << "---"
                  << "\n";
    }

    Octree::PointType pointToCheck{45, 9, 18};
    std::cout << "distance: \n";
    for (auto &elem : octree->queryRange(bbox)) {
        std::cout << "-------------\n";
        std::cout << elem << "\n";
        std::cout << "distance: " << (elem - pointToCheck).norm() << "\n";
    }
    std::cout << "end of calculating distance: \n";

    Octree::Ptr octantPtr = octree->searchElem(Octree::PointType{20, 45, 65.3});

    if (octantPtr) {
        std::cout << "found the element"
                  << "\n";
        std::cout << octantPtr->bbox() << "\n";
        std::cout << octantPtr->points()[0] << "\n";
    } else {
        std::cout << "nothing found"
                  << "\n";
    }

    Octree::VecPointType knn = octree->nearestSearch(Octree::PointType{45, 9, 18}, 4);

    std::cout << "knn result"
              << "\n";
    for (auto &elem : knn) {
        std::cout << elem << "\n";
        std::cout << "---"
                  << "\n";
    }

    octree->deleteElems(pV);
    std::cout << "after deleting\n";
    for (auto &elem : octree->points()) {
        std::cout << elem << "\n";
        std::cout << "---"
                  << "\n";
    }

    return 0;
}
