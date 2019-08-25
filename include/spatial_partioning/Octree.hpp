/**
 * @file    Octree.hpp
 *
 * @brief   header for Octree
 *
 * @author  btran
 *
 * @date    2019-06-28
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <Eigen/Eigen>

#include <string>

#include "spatial_partioning/QuadOctreeBase.hpp"

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace algo
{
template <typename DATA_TYPE, class PointContainer = Eigen::Matrix<DATA_TYPE, 3, 1>>
class Octree : public QuadOctreeBase<DATA_TYPE, 3, PointContainer>
{
 public:
    using PointType = typename QuadOctreeBase<DATA_TYPE, 3, PointContainer>::PointType;
    using VecPointType = typename QuadOctreeBase<DATA_TYPE, 3, PointContainer>::VecPointType;

    explicit Octree(const VecPointType& points, uint8_t maxLevel = 5, uint32_t maxNumPoints = 1,
                    const PointType& offsetMinPoint = PointType{std::numeric_limits<DATA_TYPE>::max(),
                                                                std::numeric_limits<DATA_TYPE>::max(),
                                                                std::numeric_limits<DATA_TYPE>::max()},
                    const PointType& offsetMaxPoint = PointType{std::numeric_limits<DATA_TYPE>::min(),
                                                                std::numeric_limits<DATA_TYPE>::min(),
                                                                std::numeric_limits<DATA_TYPE>::min()},
                    const std::string& nodeName = "Octant");

    ~Octree() = default;

 private:
    Octree(const Octree&);
    Octree& operator=(const Octree&);
};

template <typename DATA_TYPE, class PointContainer>
Octree<DATA_TYPE, PointContainer>::Octree(const VecPointType& points, uint8_t maxLevel, uint32_t maxNumPoints,
                                          const PointType& offsetMinPoint, const PointType& offsetMaxPoint,
                                          const std::string& nodeName)
    : QuadOctreeBase<DATA_TYPE, 3, PointContainer>(points, maxLevel, maxNumPoints, offsetMinPoint, offsetMaxPoint,
                                                   nodeName)
{
}

}  // namespace algo
