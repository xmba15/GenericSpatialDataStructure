/**
 * @file    Quadtree.hpp
 *
 * @brief   header for Quadtree
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
/**
 *  @brief class inherited from QuadOctreeBase to represent a Quadtree
 *
 */
template <typename DATA_TYPE, class PointContainer = Eigen::Matrix<DATA_TYPE, 2, 1>>
class Quadtree : public QuadOctreeBase<DATA_TYPE, 2, PointContainer>
{
 public:
    using PointType = typename QuadOctreeBase<DATA_TYPE, 2, PointContainer>::PointType;
    using VecPointType = typename QuadOctreeBase<DATA_TYPE, 2, PointContainer>::VecPointType;

    explicit Quadtree(const VecPointType& points, uint8_t maxLevel = 5, uint32_t maxNumPoints = 1,
                      const PointType& offsetMinPoint = PointType{std::numeric_limits<DATA_TYPE>::max(),
                                                                  std::numeric_limits<DATA_TYPE>::max()},
                      const PointType& offsetMaxPoint = PointType{std::numeric_limits<DATA_TYPE>::min(),
                                                                  std::numeric_limits<DATA_TYPE>::min()},
                      const std::string& nodeName = "Quadrant");

    ~Quadtree() = default;

 private:
    Quadtree(const Quadtree&);
    Quadtree& operator=(const Quadtree&);
};

template <typename DATA_TYPE, class PointContainer>
Quadtree<DATA_TYPE, PointContainer>::Quadtree(const VecPointType& points, uint8_t maxLevel, uint32_t maxNumPoints,
                                              const PointType& offsetMinPoint, const PointType& offsetMaxPoint,
                                              const std::string& nodeName)
    : algo::QuadOctreeBase<DATA_TYPE, 2, PointContainer>(points, maxLevel, maxNumPoints, offsetMinPoint, offsetMaxPoint,
                                                         nodeName)
{
}

}  // namespace algo
