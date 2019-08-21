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
#include <algorithm>
#include <limits>
#include <vector>

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace algo
{
template <typename DATA_TYPE, class Container = Eigen::Matrix<DATA_TYPE, 2, 1>> class Quadtree
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = Container;
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    explicit Quadtree(const VecPointType& points, uint8_t maxLevel = 5, uint32_t maxNumPoints = 1,
                      const PointType& offsetMinPoint = PointType{std::numeric_limits<DATA_TYPE>::max(),
                                                                  std::numeric_limits<DATA_TYPE>::max()},
                      const PointType& offsetMaxPoint = PointType{std::numeric_limits<DATA_TYPE>::min(),
                                                                  std::numeric_limits<DATA_TYPE>::min()});

    ~Quadtree();

 private:
    struct Quadrant {
        Quadrant();
        ~Quadrant();

        bool isLeaf;
        DATA_TYPE x, y;
        DATA_TYPE radius;

        uint32_t startIdx, endIdx;
        uint32_t numPoints;
        uint8_t level;

        Quadrant* child[4];
    };

    Quadrant* createQuadrant(DATA_TYPE x, DATA_TYPE y, DATA_TYPE radius, uint32_t startIdx, uint32_t endIdx,
                             uint32_t numPoints, uint8_t level);

    Quadtree(const Quadtree&);
    Quadtree& operator=(const Quadtree&);

 private:
    struct BoundingBox {
        PointType center;
        DATA_TYPE radius;
    };

    const BoundingBox estimateQuadraticBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                              const PointType& offsetMaxPoint) const;

 private:
    VecPointType _points;

    Quadrant* _root;

    std::vector<uint32_t> _successors;

    uint8_t _maxLevel;

    uint32_t _maxNumPoints;
};

template <typename DATA_TYPE, class Container>
Quadtree<DATA_TYPE, Container>::Quadtree(const VecPointType& points, uint8_t maxLevel, uint32_t maxNumPoints,
                                         const PointType& offsetMinPoint, const PointType& offsetMaxPoint)
    : _points(points), _maxLevel(maxLevel), _maxNumPoints(maxNumPoints)
{
    uint32_t N = this->_points.size();

    this->_successors.reserve(N);
    for (uint32_t i = 0; i < N; ++i) {
        this->_successors.emplace_back(i + 1);
    }

    const BoundingBox bbox = this->estimateQuadraticBounds(this->_points, offsetMinPoint, offsetMaxPoint);

    this->_root = this->createQuadrant(bbox.center.x(), bbox.center.y(), bbox.radius, 0, N - 1, N, 0);
}

template <typename DATA_TYPE, class Container>
typename Quadtree<DATA_TYPE, Container>::Quadrant*
Quadtree<DATA_TYPE, Container>::createQuadrant(DATA_TYPE x, DATA_TYPE y, DATA_TYPE radius, uint32_t startIdx,
                                               uint32_t endIdx, uint32_t numPoints, uint8_t level)
{
    Quadrant* quadrant = new Quadrant;
    quadrant->isLeaf = true;

    quadrant->x = x;
    quadrant->y = y;
    quadrant->radius = radius;
    quadrant->startIdx = startIdx;
    quadrant->endIdx = endIdx;
    quadrant->numPoints = numPoints;
    quadrant->level = level;

    static const DATA_TYPE factor[] = {-0.5f, 0.5f};

    if (numPoints > this->_maxNumPoints && level < this->_maxLevel) {
        quadrant->isLeaf = false;
        std::vector<uint32_t> childStarts(4, 0);
        std::vector<uint32_t> childEnds(4, 0);
        std::vector<uint32_t> childSizes(4, 0);

        uint32_t idx = startIdx;

        for (uint32_t i = 0; i < numPoints; ++i) {
            const PointType& curPoint = this->_points[idx];
            uint32_t mortonCode = 0;

            if (curPoint.x() > x) {
                mortonCode |= 1;
            }
            if (curPoint.y() > y) {
                mortonCode |= 2;
            }

            if (childSizes[mortonCode] == 0) {
                childStarts[mortonCode] = idx;
            } else {
                this->_successors[childEnds[mortonCode]] = idx;
            }

            childSizes[mortonCode] += 1;

            childEnds[mortonCode] = idx;
            idx = this->_successors[idx];
        }
    }

    return quadrant;
}

template <typename DATA_TYPE, class Container>
const typename Quadtree<DATA_TYPE, Container>::BoundingBox
Quadtree<DATA_TYPE, Container>::estimateQuadraticBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                                        const PointType& offsetMaxPoint) const
{
    BoundingBox result;
    PointType boundMinPoint = offsetMinPoint;
    PointType boundMaxPoint = offsetMaxPoint;

    for (const PointType& curP : points) {
        for (size_t i = 0; i < 2; ++i) {
            if (boundMinPoint[i] > curP[i]) {
                boundMinPoint[i] = curP[i];
            }

            if (boundMaxPoint[i] < curP[i])
                boundMaxPoint[i] = curP[i];
        }
    }

    PointType deltaPoint = boundMaxPoint - boundMinPoint;
    result.center = boundMinPoint + deltaPoint * 0.5;
    result.radius = std::max(deltaPoint.x(), deltaPoint.y());

    return result;
}

}  // namespace algo
