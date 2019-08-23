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
#include <algorithm>
#include <limits>
#include <vector>

#include <iostream>

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace algo
{
template <typename DATA_TYPE, class Container = Eigen::Matrix<DATA_TYPE, 3, 1>> class Octree
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = Container;
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    explicit Octree(const VecPointType& points, uint8_t maxLevel = 5, uint32_t maxNumPoints = 1,
                    const PointType& offsetMinPoint = PointType{std::numeric_limits<DATA_TYPE>::max(),
                                                                std::numeric_limits<DATA_TYPE>::max(),
                                                                std::numeric_limits<DATA_TYPE>::max()},
                    const PointType& offsetMaxPoint = PointType{std::numeric_limits<DATA_TYPE>::min(),
                                                                std::numeric_limits<DATA_TYPE>::min(),
                                                                std::numeric_limits<DATA_TYPE>::min()});

    ~Octree();

    std::stringstream traversal() const;

    bool insideOctreeSpace(const PointType& query) const;
    uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance = -1) const;

 private:
    struct Octant {
        Octant();
        ~Octant();

        bool isLeaf;
        DATA_TYPE x, y, z;
        DATA_TYPE radius;

        uint32_t startIdx, endIdx;
        uint32_t numPoints;
        uint8_t level;

        Octant* child[8];
    };

    struct BoundingBox {
        PointType center;
        DATA_TYPE radius;
    };

    Octant* createOctant(DATA_TYPE x, DATA_TYPE y, DATA_TYPE z, DATA_TYPE radius, uint32_t startIdx, uint32_t endIdx,
                         uint32_t numPoints, uint8_t level);

    Octree(const Octree&);
    Octree& operator=(const Octree&);

    bool overlaps(const PointType& query, DATA_TYPE radius, const Octant* octant) const;

    bool inside(const PointType& query, DATA_TYPE radius, const Octant* octant) const;
    bool inside(const PointType& query, const Octant* octant) const;

    const BoundingBox estimateCubicBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                          const PointType& offsetMaxPoint) const;

    uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance, const Octant* octant) const;

    std::stringstream traversal(const Octant* octant) const;

 private:
    VecPointType _points;

    Octant* _root;

    std::vector<uint32_t> _successors;

    uint8_t _maxLevel;

    uint32_t _maxNumPoints;
};

template <typename DATA_TYPE, class Container>
Octree<DATA_TYPE, Container>::Octree(const VecPointType& points, uint8_t maxLevel, uint32_t maxNumPoints,
                                     const PointType& offsetMinPoint, const PointType& offsetMaxPoint)
    : _points(points), _maxLevel(maxLevel), _maxNumPoints(maxNumPoints)
{
    uint32_t N = this->_points.size();

    this->_successors.reserve(N);
    for (uint32_t i = 0; i < N; ++i) {
        this->_successors.emplace_back(i + 1);
    }

    const BoundingBox bbox = this->estimateCubicBounds(this->_points, offsetMinPoint, offsetMaxPoint);

    this->_root = this->createOctant(bbox.center.x(), bbox.center.y(), bbox.center.z(), bbox.radius, 0, N - 1, N, 0);
}

template <typename DATA_TYPE, class Container> Octree<DATA_TYPE, Container>::~Octree()
{
    delete _root;
}

template <typename DATA_TYPE, class Container>
bool Octree<DATA_TYPE, Container>::insideOctreeSpace(const PointType& query) const
{
    return this->inside(query, this->_root);
}

template <typename DATA_TYPE, class Container>
typename Octree<DATA_TYPE, Container>::Octant*
Octree<DATA_TYPE, Container>::createOctant(DATA_TYPE x, DATA_TYPE y, DATA_TYPE z, DATA_TYPE radius, uint32_t startIdx,
                                           uint32_t endIdx, uint32_t numPoints, uint8_t level)
{
    Octant* octant = new Octant;
    octant->isLeaf = true;

    octant->x = x;
    octant->y = y;
    octant->z = z;
    octant->radius = radius;
    octant->startIdx = startIdx;
    octant->endIdx = endIdx;
    octant->numPoints = numPoints;
    octant->level = level;

    static const DATA_TYPE factor[] = {-0.5f, 0.5f};

    if (numPoints > this->_maxNumPoints && level < this->_maxLevel) {
        octant->isLeaf = false;
        std::vector<uint32_t> childStarts(8, 0);
        std::vector<uint32_t> childEnds(8, 0);
        std::vector<uint32_t> childSizes(8, 0);

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

            if (curPoint.z() > z) {
                mortonCode |= 4;
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

        for (uint32_t i = 0; i < 8; ++i) {
            if (childSizes[i] == 0) {
                continue;
            }

            DATA_TYPE childX = x + factor[(i & 1) > 0] * radius;
            DATA_TYPE childY = y + factor[(i & 2) > 0] * radius;
            DATA_TYPE childZ = z + factor[(i & 4) > 0] * radius;

            octant->child[i] = this->createOctant(childX, childY, childZ, childRadius, childStarts[i], childEnds[i],
                                                  childSizes[i], level + 1);

            if (firstTime) {
                octant->startIdx = octant->child[i]->startIdx;
            } else {
                this->_successors[octant->child[lastChildIdx]->endIdx] = octant->child[i]->startIdx;
            }

            lastChildIdx = i;
            octant->endIdx = octant->child[i]->endIdx;
            firstTime = false;
        }
    }

    return octant;
}

template <typename DATA_TYPE, class Container>
bool Octree<DATA_TYPE, Container>::overlaps(const PointType& query, DATA_TYPE radius, const Octant* octant) const
{
    DATA_TYPE deltaX = query.x() - octant->x;
    DATA_TYPE deltaY = query.y() - octant->y;
    DATA_TYPE deltaZ = query.z() - octant->z;

    deltaX = std::abs(deltaX);
    deltaY = std::abs(deltaY);
    deltaZ = std::abs(deltaZ);

    DATA_TYPE maxDist = radius + octant->radius;

    if (deltaX > maxDist || deltaY > maxDist || deltaZ > maxDist) {
        return false;
    }

    if (deltaX < octant->radius || deltaY < octant->radius || deltaZ < octant->radius) {
        return true;
    }

    deltaX = std::max(deltaX - octant->radius, static_cast<DATA_TYPE>(0.0f));
    deltaY = std::max(deltaY - octant->radius, static_cast<DATA_TYPE>(0.0f));
    deltaZ = std::max(deltaZ - octant->radius, static_cast<DATA_TYPE>(0.0f));

    return (std::sqrt(std::pow(deltaX, 2) + std::pow(deltaY, 2) + std::pow(deltaZ, 2)) < radius);
}

template <typename DATA_TYPE, class Container>
bool Octree<DATA_TYPE, Container>::inside(const PointType& query, DATA_TYPE radius, const Octant* octant) const
{
    DATA_TYPE deltaX = query.x() - octant->x;
    DATA_TYPE deltaY = query.y() - octant->y;
    DATA_TYPE deltaZ = query.z() - octant->z;

    deltaX = std::abs(deltaX) + radius;
    deltaY = std::abs(deltaY) + radius;
    deltaZ = std::abs(deltaY) + radius;

    if (deltaX > octant->radius || deltaY > octant->radius || deltaZ > octant->radius) {
        return false;
    }

    return true;
}

template <typename DATA_TYPE, class Container>
bool Octree<DATA_TYPE, Container>::inside(const PointType& query, const Octant* octant) const
{
    DATA_TYPE deltaX = query.x() - octant->x;
    DATA_TYPE deltaY = query.y() - octant->y;
    DATA_TYPE deltaZ = query.z() - octant->z;

    deltaX = std::abs(deltaX);
    deltaY = std::abs(deltaY);
    deltaZ = std::abs(deltaY);

    if (deltaX > octant->radius || deltaY > octant->radius || deltaZ > octant->radius) {
        return false;
    }

    return true;
}

template <typename DATA_TYPE, class Container>
const typename Octree<DATA_TYPE, Container>::BoundingBox
Octree<DATA_TYPE, Container>::estimateCubicBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                                  const PointType& offsetMaxPoint) const
{
    BoundingBox result;
    PointType boundMinPoint = offsetMinPoint;
    PointType boundMaxPoint = offsetMaxPoint;

    for (const PointType& curP : points) {
        for (size_t i = 0; i < 3; ++i) {
            if (boundMinPoint[i] > curP[i]) {
                boundMinPoint[i] = curP[i];
            }

            if (boundMaxPoint[i] < curP[i])
                boundMaxPoint[i] = curP[i];
        }
    }

    PointType deltaPoint = boundMaxPoint - boundMinPoint;
    result.center = boundMinPoint + deltaPoint * 0.5;
    result.radius = 0.5 * std::max(std::max(deltaPoint.x(), deltaPoint.y()), deltaPoint.z());

    return result;
}

template <typename DATA_TYPE, class Container> std::stringstream Octree<DATA_TYPE, Container>::traversal() const
{
    return traversal(this->_root);
}

template <typename DATA_TYPE, class Container>
std::stringstream Octree<DATA_TYPE, Container>::traversal(const Octant* octant) const
{
    std::stringstream ss;

    if (octant->isLeaf) {
        uint32_t idx = octant->startIdx;
        ss << "----------------------------\n";
        ss << "octant: "
           << "\nlevel: " << int(octant->level) << "\nx: " << octant->x << "\ny: " << octant->y << "\nz: " << octant->z
           << "\nradius: " << octant->radius << "\n";

        for (uint32_t i = 0; i < octant->numPoints; ++i) {
            ss << "Point: \n";
            ss << this->_points[idx];
            ss << "\n";
            idx = this->_successors[idx];
        }
        ss << "----------------------------\n";
        return ss;
    }

    for (size_t i = 0; i < 8; ++i) {
        if (octant->child[i] != 0) {
            ss << traversal(octant->child[i]).str();
        }
    }

    return ss;
}

template <typename DATA_TYPE, class Container>
uint32_t Octree<DATA_TYPE, Container>::findNeighbor(const PointType& query, DATA_TYPE minDistance) const
{
    return this->findNeighbor(query, minDistance, this->_root);
}

template <typename DATA_TYPE, class Container>
uint32_t Octree<DATA_TYPE, Container>::findNeighbor(const PointType& query, DATA_TYPE minDistance,
                                                    const Octant* octant) const
{
    uint32_t resultIdx = -1;

    DATA_TYPE maxDistance = std::numeric_limits<DATA_TYPE>::max();
    std::vector<const Octant*, Eigen::aligned_allocator<const Octant*>> octantPtrs;

    octantPtrs.emplace_back(octant);

    while (!octantPtrs.empty()) {
        const Octant* curOctant = octantPtrs.back();
        octantPtrs.pop_back();

        if (!this->overlaps(query, maxDistance, curOctant)) {
            continue;
        }

        if (curOctant->isLeaf) {
            uint32_t idx = curOctant->startIdx;

            for (uint32_t i = 0; i < curOctant->numPoints; ++i) {
                const PointType& curPoint = this->_points[idx];
                const DATA_TYPE delta = (query - curPoint).norm();

                if (delta > minDistance && delta < maxDistance) {
                    resultIdx = idx;
                    maxDistance = delta;
                }

                idx = this->_successors[idx];
            }
        }

        for (size_t i = 0; i < 8; ++i) {
            if (curOctant->child[i] != 0) {
                octantPtrs.emplace_back(curOctant->child[i]);
            }
        }
    }

    return resultIdx;
}

template <typename DATA_TYPE, class Container>
Octree<DATA_TYPE, Container>::Octant::Octant()
    : isLeaf(true), x(0.0f), y(0.0f), z(0.0f), radius(0.0f), startIdx(0), endIdx(0), numPoints(0)
{
    memset(&child, 0, 8 * sizeof(Octant*));
}

template <typename DATA_TYPE, class Container> Octree<DATA_TYPE, Container>::Octant::~Octant()
{
    for (uint32_t i = 0; i < 8; ++i) {
        delete child[i];
    }
}

}  // namespace algo
