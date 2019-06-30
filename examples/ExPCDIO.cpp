/**
 * @file    ExPCDIO.cpp
 *
 * @author  btran
 *
 * @date    2019-09-04
 *
 * Copyright (c) organization
 *
 */

#include <iostream>

#include <spatial_partioning/PointCloud.hpp>
#include <spatial_partioning/utility/PCDInOut.hpp>

#ifndef WITH_VISUALIZATION
#define ENABLE_VISUALIZATION 1
#endif  // WITH_VISUALIZATION

#if ENABLE_VISUALIZATION
#include <matplotlib_cpp/MatplotlibCpp.hpp>
// import modules of matplotlib library
static pe::vis::Matplotlib mpllib;
#endif  // ENABLE_VISUALIZATION

using Octree = algo::Octree<double, algo::geometry::PointCloud::PointType>;
using OctreePtr = Octree*;
static OctreePtr octreePtr;

void drawFunc(const std::array<double, 3>& center, const double radius);

int main(int argc, char* argv[])
{
#ifdef DATA_PATH
    std::stringstream ss;
    ss << DATA_PATH << "/easy_jet.pcd";
    algo::geometry::PointCloud pointcloud;
    algo::pcdio::ReadPointCloud(ss.str(), pointcloud, "auto", true, true, true);
    std::cout << pointcloud.points_.size() << "\n";

    octreePtr = new Octree(pointcloud.points_);

#if ENABLE_VISUALIZATION

    // check if the modules are imported successully or not
    if (!mpllib.imported()) {
        std::cout << "Failed to import matplotlib library\n";
        exit(EXIT_FAILURE);
    }

    const auto xyzS = pointcloud.extractPointDataEachAxis();

    mpllib.initializeAxes3D();

    mpllib.scatterAxes3D(xyzS[0], xyzS[1], xyzS[2]);

    octreePtr->drawOctree(drawFunc);

    delete octreePtr;

    mpllib.savefig("easy_jet.png");

    mpllib.show();
#endif  // ENABLE_VISUALIZATION

#endif  // DATA_PATH

    return EXIT_FAILURE;
}

void drawFunc(const std::array<double, 3>& center, const double radius)
{
    std::array<double, 3> vertices[] = {
        {center[0] + radius, center[1] - radius, center[2] - radius},
        {center[0] + radius, center[1] + radius, center[2] - radius},

        {center[0] - radius, center[1] + radius, center[2] - radius},
        {center[0] - radius, center[1] - radius, center[2] - radius},

        {center[0] + radius, center[1] - radius, center[2] + radius},
        {center[0] + radius, center[1] + radius, center[2] + radius},

        {center[0] - radius, center[1] - radius, center[2] + radius},
        {center[0] - radius, center[1] + radius, center[2] + radius},
    };

    std::array<int, 2> edges[] = {{0, 1}, {0, 3}, {0, 4}, {2, 1}, {2, 3}, {2, 7},
                                  {6, 3}, {6, 4}, {6, 7}, {5, 1}, {5, 4}, {5, 7}};

    for (const auto& edge : edges) {
        const auto firstPoint = vertices[edge[0]];
        const auto secondPoint = vertices[edge[1]];

        mpllib.plotAxes3D(std::vector<double>{firstPoint[0], secondPoint[0]},
                          std::vector<double>{firstPoint[1], secondPoint[1]},
                          std::vector<double>{firstPoint[2], secondPoint[2]},
                          {
                              {"color", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("r")},
                              {"linestyle", pe::vis::Matplotlib::createAnyBaseMapData<std::string>("solid")},
                          });
    }
}
