/**
 * @file    QuadOctreeBase.hpp
 *
 * @date    2019-08-24
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace algo
{
template <typename DATA_TYPE, size_t POINT_DIMENSION,
          class PointContainer = Eigen::Matrix<DATA_TYPE, POINT_DIMENSION, 1>>
class QuadOctreeBase
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = PointContainer;

    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    using PointCoordinates = std::array<DATA_TYPE, POINT_DIMENSION>;

    static constexpr size_t NUM_CHILD = std::pow(2, POINT_DIMENSION);

    QuadOctreeBase(const VecPointType& points, uint8_t maxLevel, uint32_t maxNumPoints, const PointType& offsetMinPoint,
                   const PointType& offsetMaxPoint, const std::string& nodeName = "QuadOctreeBaseNode");

    ~QuadOctreeBase();

    const VecPointType& points() const;

    virtual std::stringstream traversal() const;
    virtual bool insideSpace(const PointType& query) const;
    virtual uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance = -1) const;
    virtual std::vector<uint32_t> knn(const PointType& query, uint32_t k, DATA_TYPE minDistance = -1) const;

 private:
    struct QuadOctreeBaseNode {
        QuadOctreeBaseNode();
        ~QuadOctreeBaseNode();

        bool isLeaf;
        PointCoordinates coordinates;
        DATA_TYPE radius;

        uint32_t startIdx, endIdx;
        uint32_t numPoints;
        uint8_t level;

        QuadOctreeBaseNode* child[NUM_CHILD];
    };

    struct BoundingBox {
        PointCoordinates coordinates;
        DATA_TYPE radius;
    };

    QuadOctreeBaseNode* createQuadOctreeBaseNode(const PointCoordinates& coordinates, DATA_TYPE radius,
                                                 uint32_t startIdx, uint32_t endIdx, uint32_t numPoints, uint8_t level);

    QuadOctreeBase(const QuadOctreeBase&);
    QuadOctreeBase& operator=(const QuadOctreeBase&);

    bool overlaps(const PointType& query, DATA_TYPE radius, const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    bool inside(const PointType& query, DATA_TYPE radius, const QuadOctreeBaseNode* quadOctreeBaseNode) const;
    bool inside(const PointType& query, const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    const BoundingBox estimateBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                     const PointType& offsetMaxPoint) const;

    uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance,
                          const QuadOctreeBaseNode* quadOctreeBaseNode) const;
    std::vector<uint32_t> knn(const PointType& query, uint32_t k, DATA_TYPE minDistance,
                              const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    std::stringstream traversal(const QuadOctreeBaseNode* quadOctreeBaseNode) const;

 protected:
    VecPointType _points;

    QuadOctreeBaseNode* _root;

    std::vector<uint32_t> _successors;

    uint8_t _maxLevel;

    uint32_t _maxNumPoints;

    std::string _nodeName;
};

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::QuadOctreeBase(const VecPointType& points, uint8_t maxLevel,
                                                                           uint32_t maxNumPoints,
                                                                           const PointType& offsetMinPoint,
                                                                           const PointType& offsetMaxPoint,
                                                                           const std::string& nodeName)
    : _points(points), _maxLevel(maxLevel), _maxNumPoints(maxNumPoints), _nodeName(nodeName)
{
    const uint32_t N = this->_points.size();

    this->_successors.reserve(N);
    for (uint32_t i = 0; i < N; ++i) {
        this->_successors.emplace_back(i + 1);
    }

    const BoundingBox bbox = this->estimateBounds(this->_points, offsetMinPoint, offsetMaxPoint);

    this->_root = this->createQuadOctreeBaseNode(bbox.coordinates, bbox.radius, 0, N - 1, N, 0);
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::~QuadOctreeBase()
{
    delete _root;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
const typename QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::VecPointType&
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::points() const
{
    return this->_points;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
bool QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::insideSpace(const PointType& query) const
{
    return this->inside(query, this->_root);
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
typename QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::QuadOctreeBaseNode*
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::createQuadOctreeBaseNode(
    const PointCoordinates& coordinates, DATA_TYPE radius, uint32_t startIdx, uint32_t endIdx, uint32_t numPoints,
    uint8_t level)
{
    QuadOctreeBaseNode* quadOctreeBaseNode = new QuadOctreeBaseNode;
    quadOctreeBaseNode->isLeaf = true;

    quadOctreeBaseNode->coordinates = coordinates;
    quadOctreeBaseNode->radius = radius;
    quadOctreeBaseNode->startIdx = startIdx;
    quadOctreeBaseNode->endIdx = endIdx;
    quadOctreeBaseNode->numPoints = numPoints;
    quadOctreeBaseNode->level = level;

    static const DATA_TYPE factor[] = {-0.5f, 0.5f};

    if (numPoints > this->_maxNumPoints && level < this->_maxLevel) {
        quadOctreeBaseNode->isLeaf = false;
        std::vector<uint32_t> childStarts(NUM_CHILD, 0);
        std::vector<uint32_t> childEnds(NUM_CHILD, 0);
        std::vector<uint32_t> childSizes(NUM_CHILD, 0);

        uint32_t idx = startIdx;

        for (uint32_t i = 0; i < numPoints; ++i) {
            const PointType& curPoint = this->_points[idx];
            uint32_t mortonCode = 0;

            for (int i = 0; i < POINT_DIMENSION; ++i) {
                if (curPoint[i] > coordinates[i]) {
                    mortonCode |= (1 << i);
                }
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

        for (uint32_t i = 0; i < NUM_CHILD; ++i) {
            if (childSizes[i] == 0) {
                continue;
            }

            PointCoordinates childCoordinates;
            for (int j = 0; j < POINT_DIMENSION; ++j) {
                childCoordinates[j] = coordinates[j] + factor[(i & (1 << j)) > 0] * radius;
            }

            quadOctreeBaseNode->child[i] = this->createQuadOctreeBaseNode(childCoordinates, childRadius, childStarts[i],
                                                                          childEnds[i], childSizes[i], level + 1);

            if (firstTime) {
                quadOctreeBaseNode->startIdx = quadOctreeBaseNode->child[i]->startIdx;
            } else {
                this->_successors[quadOctreeBaseNode->child[lastChildIdx]->endIdx] =
                    quadOctreeBaseNode->child[i]->startIdx;
            }

            lastChildIdx = i;
            quadOctreeBaseNode->endIdx = quadOctreeBaseNode->child[i]->endIdx;
            firstTime = false;
        }
    }

    return quadOctreeBaseNode;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
bool QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::overlaps(
    const PointType& query, DATA_TYPE radius, const QuadOctreeBaseNode* quadOctreeBaseNode) const
{
    DATA_TYPE maxDist = radius + quadOctreeBaseNode->radius;

    std::array<DATA_TYPE, POINT_DIMENSION> delta;
    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        delta[i] = query[i] - quadOctreeBaseNode->coordinates[i];
        delta[i] = std::abs(delta[i]);

        if (delta[i] > maxDist) {
            return false;
        }
    }

    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        if (delta[i] < quadOctreeBaseNode->radius) {
            return true;
        }
    }

    DATA_TYPE sqrDistToEdge = 0.0;
    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        delta[i] = std::max(delta[i] - quadOctreeBaseNode->radius, static_cast<DATA_TYPE>(0.0f));
        sqrDistToEdge += std::pow(delta[i], 2);
    }

    return (std::sqrt(sqrDistToEdge) < radius);
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
bool QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::inside(
    const PointType& query, DATA_TYPE radius, const QuadOctreeBaseNode* quadOctreeBaseNode) const
{
    std::array<DATA_TYPE, POINT_DIMENSION> delta;
    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        DATA_TYPE iDelta = query[i] - quadOctreeBaseNode->coordinates[i];
        iDelta = std::abs(iDelta) + radius;

        if (iDelta > quadOctreeBaseNode->radius) {
            return false;
        }
    }

    return true;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
bool QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::inside(
    const PointType& query, const QuadOctreeBaseNode* quadOctreeBaseNode) const
{
    for (size_t i = 0; i < POINT_DIMENSION; ++i) {
        DATA_TYPE iDelta = query[i] - quadOctreeBaseNode->coordinates[i];
        iDelta = std::abs(iDelta);

        if (iDelta > quadOctreeBaseNode->radius) {
            return false;
        }
    }

    return true;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
const typename QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::BoundingBox
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::estimateBounds(const VecPointType& points,
                                                                           const PointType& offsetMinPoint,
                                                                           const PointType& offsetMaxPoint) const
{
    BoundingBox result;
    PointType boundMinPoint = offsetMinPoint;
    PointType boundMaxPoint = offsetMaxPoint;

    for (const PointType& curP : points) {
        for (size_t i = 0; i < POINT_DIMENSION; ++i) {
            if (boundMinPoint[i] > curP[i]) {
                boundMinPoint[i] = curP[i];
            }

            if (boundMaxPoint[i] < curP[i])
                boundMaxPoint[i] = curP[i];
        }
    }

    PointType deltaPoint = boundMaxPoint - boundMinPoint;
    PointType center = boundMinPoint + deltaPoint * 0.5;

    assert(center.size() == POINT_DIMENSION);
    std::copy(center.data(), center.data() + POINT_DIMENSION, result.coordinates.begin());

    DATA_TYPE maxDimension = std::numeric_limits<DATA_TYPE>::min();
    for (int i = 0; i < POINT_DIMENSION; ++i) {
        maxDimension = std::max(maxDimension, deltaPoint[i]);
    }

    result.radius = 0.5 * maxDimension;

    return result;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
std::stringstream QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::traversal() const
{
    return traversal(this->_root);
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
std::stringstream QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::traversal(
    const QuadOctreeBaseNode* quadOctreeBaseNode) const
{
    std::stringstream ss;

    if (quadOctreeBaseNode->isLeaf) {
        uint32_t idx = quadOctreeBaseNode->startIdx;
        ss << "----------------------------\n";
        ss << this->_nodeName << ": "
           << "\nlevel: " << int(quadOctreeBaseNode->level) << "\nradius: " << quadOctreeBaseNode->radius << "\n";

        for (size_t i = 0; i < POINT_DIMENSION; ++i) {
            ss << i << " axis: " << quadOctreeBaseNode->coordinates[i] << "\n";
        }

        for (uint32_t i = 0; i < quadOctreeBaseNode->numPoints; ++i) {
            ss << "Point: \n";
            ss << this->_points[idx];
            ss << "\n";
            idx = this->_successors[idx];
        }
        ss << "----------------------------\n";
        return ss;
    }

    for (size_t i = 0; i < NUM_CHILD; ++i) {
        if (quadOctreeBaseNode->child[i] != 0) {
            ss << traversal(quadOctreeBaseNode->child[i]).str();
        }
    }

    return ss;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
uint32_t QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::findNeighbor(const PointType& query,
                                                                                  DATA_TYPE minDistance) const
{
    return this->findNeighbor(query, minDistance, this->_root);
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
uint32_t QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::findNeighbor(
    const PointType& query, DATA_TYPE minDistance, const QuadOctreeBaseNode* quadOctreeBaseNode) const
{
    uint32_t resultIdx = -1;

    DATA_TYPE maxDistance = std::numeric_limits<DATA_TYPE>::max();
    std::vector<const QuadOctreeBaseNode*, Eigen::aligned_allocator<const QuadOctreeBaseNode*>> quadOctreeBaseNodePtrs;

    quadOctreeBaseNodePtrs.emplace_back(quadOctreeBaseNode);

    while (!quadOctreeBaseNodePtrs.empty()) {
        const QuadOctreeBaseNode* curQuadOctreeBaseNode = quadOctreeBaseNodePtrs.back();
        quadOctreeBaseNodePtrs.pop_back();

        if (!this->overlaps(query, maxDistance, curQuadOctreeBaseNode)) {
            continue;
        }

        if (curQuadOctreeBaseNode->isLeaf) {
            uint32_t idx = curQuadOctreeBaseNode->startIdx;

            for (uint32_t i = 0; i < curQuadOctreeBaseNode->numPoints; ++i) {
                const PointType& curPoint = this->_points[idx];
                const DATA_TYPE delta = (query - curPoint).norm();

                if (delta > minDistance && delta < maxDistance) {
                    resultIdx = idx;
                    maxDistance = delta;
                }

                idx = this->_successors[idx];
            }
        }

        for (size_t i = 0; i < NUM_CHILD; ++i) {
            if (curQuadOctreeBaseNode->child[i] != 0) {
                quadOctreeBaseNodePtrs.emplace_back(curQuadOctreeBaseNode->child[i]);
            }
        }
    }

    return resultIdx;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
std::vector<uint32_t> QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::knn(const PointType& query,
                                                                                      uint32_t k,
                                                                                      DATA_TYPE minDistance) const
{
    return this->knn(query, k, minDistance, this->_root);
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
std::vector<uint32_t> QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::knn(
    const PointType& query, uint32_t k, DATA_TYPE minDistance, const QuadOctreeBaseNode* quadOctreeBaseNode) const
{
    std::vector<uint32_t> resultIndices;

    if (k >= this->_points.size()) {
        resultIndices.resize(this->_points.size());
        std::iota(resultIndices.begin(), resultIndices.end(), 0);
        std::sort(resultIndices.begin(), resultIndices.end(), [&query, this](const uint32_t idx1, const uint32_t idx2) {
            return (this->_points[idx1] - query).norm() < (this->_points[idx2] - query).norm();
        });

        return resultIndices;
    }

    DATA_TYPE maxDistance = std::numeric_limits<DATA_TYPE>::max();
    std::vector<const QuadOctreeBaseNode*, Eigen::aligned_allocator<const QuadOctreeBaseNode*>> quadOctreeBaseNodePtrs;

    //! DistanceIndex -> (distance from point P_i to query point, index of P_i in this->_points)
    using DistanceIndex = std::pair<DATA_TYPE, uint32_t>;

    std::priority_queue<DistanceIndex, std::vector<DistanceIndex>, std::less<DistanceIndex>> maxHeap;

    quadOctreeBaseNodePtrs.emplace_back(quadOctreeBaseNode);
    while (!quadOctreeBaseNodePtrs.empty()) {
        const QuadOctreeBaseNode* curQuadOctreeBaseNode = quadOctreeBaseNodePtrs.back();
        quadOctreeBaseNodePtrs.pop_back();

        if (!this->overlaps(query, maxDistance, curQuadOctreeBaseNode)) {
            continue;
        }

        if (curQuadOctreeBaseNode->isLeaf) {
            uint32_t idx = curQuadOctreeBaseNode->startIdx;

            for (uint32_t i = 0; i < curQuadOctreeBaseNode->numPoints; ++i) {
                const PointType& curPoint = this->_points[idx];
                const DATA_TYPE delta = (query - curPoint).norm();

                if (delta > minDistance) {
                    maxHeap.emplace(std::make_pair(delta, idx));
                }

                idx = this->_successors[idx];
            }
        }

        if (maxHeap.size() > k) {
            while (maxHeap.size() > k) {
                maxHeap.pop();
            }
        }

        if (maxHeap.size() == k) {
            maxDistance = maxHeap.top().first;
        }

        for (size_t i = 0; i < NUM_CHILD; ++i) {
            if (curQuadOctreeBaseNode->child[i] != 0) {
                quadOctreeBaseNodePtrs.emplace_back(curQuadOctreeBaseNode->child[i]);
            }
        }
    }

    resultIndices.reserve(maxHeap.size());
    while (!maxHeap.empty()) {
        resultIndices.emplace_back(maxHeap.top().second);
        maxHeap.pop();
    }

    std::reverse(resultIndices.begin(), resultIndices.end());

    return resultIndices;
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::QuadOctreeBaseNode::QuadOctreeBaseNode()
    : isLeaf(true), radius(0.0f), startIdx(0), endIdx(0), numPoints(0)
{
    this->coordinates.fill(0.0f);

    memset(&child, 0, NUM_CHILD * sizeof(QuadOctreeBaseNode*));
}

template <typename DATA_TYPE, size_t POINT_DIMENSION, class PointContainer>
QuadOctreeBase<DATA_TYPE, POINT_DIMENSION, PointContainer>::QuadOctreeBaseNode::~QuadOctreeBaseNode()
{
    for (uint32_t i = 0; i < NUM_CHILD; ++i) {
        delete child[i];
    }
}

}  // namespace algo
