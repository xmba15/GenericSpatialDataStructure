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
#endif  // ENABLE_VISUALIZATION

int main(int argc, char* argv[])
{
#ifdef DATA_PATH
    std::stringstream ss;
    ss << DATA_PATH << "/easy_jet.pcd";
    algo::geometry::PointCloud pointcloud;
    algo::pcdio::ReadPointCloud(ss.str(), pointcloud, "auto", true, true, true);
    std::cout << pointcloud.points_.size() << "\n";

#if ENABLE_VISUALIZATION
    // import modules of matplotlib library
    pe::vis::Matplotlib mpllib;

    // check if the modules are imported successully or not
    if (!mpllib.imported()) {
        std::cout << "Failed to import matplotlib library\n";
        exit(EXIT_FAILURE);
    }

    const auto xyzS = pointcloud.extractPointDataEachAxis();

    mpllib.initializeAxes3D();

    mpllib.scatterAxes3D(xyzS[0], xyzS[1], xyzS[2]);

    mpllib.savefig("easy_jet.png");

    mpllib.show();
#endif  // ENABLE_VISUALIZATION

#endif  // DATA_PATH

    return EXIT_FAILURE;
}
