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

#include <iostream>

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

    std::stringstream traversal() const;

    bool insideQuadtreeSpace(const PointType& query) const;
    uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance = -1) const;

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

    struct BoundingBox {
        PointType center;
        DATA_TYPE radius;
    };

    Quadrant* createQuadrant(DATA_TYPE x, DATA_TYPE y, DATA_TYPE radius, uint32_t startIdx, uint32_t endIdx,
                             uint32_t numPoints, uint8_t level);

    Quadtree(const Quadtree&);
    Quadtree& operator=(const Quadtree&);

    bool overlaps(const PointType& query, DATA_TYPE radius, const Quadrant* quadrant) const;

    bool inside(const PointType& query, DATA_TYPE radius, const Quadrant* quadrant) const;
    bool inside(const PointType& query, const Quadrant* quadrant) const;

    const BoundingBox estimateQuadraticBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                              const PointType& offsetMaxPoint) const;

    uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance, const Quadrant* quadrant) const;

    std::stringstream traversal(const Quadrant* quadrant) const;

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

template <typename DATA_TYPE, class Container> Quadtree<DATA_TYPE, Container>::~Quadtree()
{
    delete _root;
}

template <typename DATA_TYPE, class Container>
bool Quadtree<DATA_TYPE, Container>::insideQuadtreeSpace(const PointType& query) const
{
    return this->inside(query, this->_root);
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

        const DATA_TYPE childRadius = 0.5f * radius;
        bool firstTime = true;
        uint32_t lastChildIdx = 0;

        for (uint32_t i = 0; i < 4; ++i) {
            if (childSizes[i] == 0) {
                continue;
            }

            DATA_TYPE childX = x + factor[(i & 1) > 0] * radius;
            DATA_TYPE childY = y + factor[(i & 2) > 0] * radius;

            quadrant->child[i] = this->createQuadrant(childX, childY, childRadius, childStarts[i], childEnds[i],
                                                      childSizes[i], level + 1);

            if (firstTime) {
                quadrant->startIdx = quadrant->child[i]->startIdx;
            } else {
                this->_successors[quadrant->child[lastChildIdx]->endIdx] = quadrant->child[i]->startIdx;
            }

            lastChildIdx = i;
            quadrant->endIdx = quadrant->child[i]->endIdx;
            firstTime = false;
        }
    }

    return quadrant;
}

template <typename DATA_TYPE, class Container>
bool Quadtree<DATA_TYPE, Container>::overlaps(const PointType& query, DATA_TYPE radius, const Quadrant* quadrant) const
{
    DATA_TYPE deltaX = query.x() - quadrant->x;
    DATA_TYPE deltaY = query.y() - quadrant->y;

    deltaX = std::abs(deltaX);
    deltaY = std::abs(deltaY);

    DATA_TYPE maxDist = radius + quadrant->radius;

    if (deltaX > maxDist || deltaY > maxDist) {
        return false;
    }

    if (deltaX < quadrant->radius || deltaY < quadrant->radius) {
        return true;
    }

    deltaX = std::max(deltaX - quadrant->radius, static_cast<DATA_TYPE>(0.0f));
    deltaY = std::max(deltaY - quadrant->radius, static_cast<DATA_TYPE>(0.0f));

    return (std::sqrt(std::pow(deltaX, 2) + std::pow(deltaY, 2)) < radius);
}

template <typename DATA_TYPE, class Container>
bool Quadtree<DATA_TYPE, Container>::inside(const PointType& query, DATA_TYPE radius, const Quadrant* quadrant) const
{
    DATA_TYPE deltaX = query.x() - quadrant->x;
    DATA_TYPE deltaY = query.y() - quadrant->y;

    deltaX = std::abs(deltaX) + radius;
    deltaY = std::abs(deltaY) + radius;

    if (deltaX > quadrant->radius || deltaY > quadrant->radius) {
        return false;
    }

    return true;
}

template <typename DATA_TYPE, class Container>
bool Quadtree<DATA_TYPE, Container>::inside(const PointType& query, const Quadrant* quadrant) const
{
    DATA_TYPE deltaX = query.x() - quadrant->x;
    DATA_TYPE deltaY = query.y() - quadrant->y;

    deltaX = std::abs(deltaX);
    deltaY = std::abs(deltaY);

    if (deltaX > quadrant->radius || deltaY > quadrant->radius) {
        return false;
    }

    return true;
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
    result.radius = 0.5 * std::max(deltaPoint.x(), deltaPoint.y());

    return result;
}

template <typename DATA_TYPE, class Container> std::stringstream Quadtree<DATA_TYPE, Container>::traversal() const
{
    return traversal(this->_root);
}

template <typename DATA_TYPE, class Container>
std::stringstream Quadtree<DATA_TYPE, Container>::traversal(const Quadrant* quadrant) const
{
    std::stringstream ss;

    if (quadrant->isLeaf) {
        uint32_t idx = quadrant->startIdx;
        ss << "----------------------------\n";
        ss << "quadrant: "
           << "\nlevel: " << int(quadrant->level) << "\nx: " << quadrant->x << "\ny: " << quadrant->y
           << "\nradius: " << quadrant->radius << "\n";

        for (uint32_t i = 0; i < quadrant->numPoints; ++i) {
            ss << "Point: \n";
            ss << this->_points[idx];
            ss << "\n";
            idx = this->_successors[idx];
        }
        ss << "----------------------------\n";
        return ss;
    }

    for (size_t i = 0; i < 4; ++i) {
        if (quadrant->child[i] != 0) {
            ss << traversal(quadrant->child[i]).str();
        }
    }

    return ss;
}

template <typename DATA_TYPE, class Container>
uint32_t Quadtree<DATA_TYPE, Container>::findNeighbor(const PointType& query, DATA_TYPE minDistance) const
{
    return this->findNeighbor(query, minDistance, this->_root);
}

template <typename DATA_TYPE, class Container>
uint32_t Quadtree<DATA_TYPE, Container>::findNeighbor(const PointType& query, DATA_TYPE minDistance,
                                                      const Quadrant* quadrant) const
{
    uint32_t resultIdx = -1;

    DATA_TYPE maxDistance = std::numeric_limits<DATA_TYPE>::max();
    std::vector<const Quadrant*, Eigen::aligned_allocator<const Quadrant*>> quadrantPtrs;

    quadrantPtrs.emplace_back(quadrant);

    while (!quadrantPtrs.empty()) {
        const Quadrant* curQuadrant = quadrantPtrs.back();
        quadrantPtrs.pop_back();

        if (!this->overlaps(query, maxDistance, curQuadrant)) {
            continue;
        }

        if (curQuadrant->isLeaf) {
            uint32_t idx = curQuadrant->startIdx;

            for (uint32_t i = 0; i < curQuadrant->numPoints; ++i) {
                const PointType& curPoint = this->_points[idx];
                const DATA_TYPE delta = (query - curPoint).norm();

                if (delta > minDistance && delta < maxDistance) {
                    resultIdx = idx;
                    maxDistance = delta;
                }

                idx = this->_successors[idx];
            }
        }

        for (size_t i = 0; i < 4; ++i) {
            if (curQuadrant->child[i] != 0) {
                quadrantPtrs.emplace_back(curQuadrant->child[i]);
            }
        }
    }

    return resultIdx;
}

template <typename DATA_TYPE, class Container>
Quadtree<DATA_TYPE, Container>::Quadrant::Quadrant()
    : isLeaf(true), x(0.0f), y(0.0f), radius(0.0f), startIdx(0), endIdx(0), numPoints(0)
{
    memset(&child, 0, 4 * sizeof(Quadrant*));
}

template <typename DATA_TYPE, class Container> Quadtree<DATA_TYPE, Container>::Quadrant::~Quadrant()
{
    for (uint32_t i = 0; i < 4; ++i) {
        delete child[i];
    }
}

}  // namespace algo
