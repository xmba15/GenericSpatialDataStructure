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
/**
 *  @brief Base Class for both Quadtree and Octree
 *
 */
template <typename DATA_TYPE, size_t POINT_DIMENSION,
          class PointContainer = Eigen::Matrix<DATA_TYPE, POINT_DIMENSION, 1>>
class QuadOctreeBase
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = PointContainer;

    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    //! array type to store point coordinates
    using PointCoordinates = std::array<DATA_TYPE, POINT_DIMENSION>;

    //! constant number of children stored in each node
    static constexpr size_t NUM_CHILD = std::pow(2, POINT_DIMENSION);

    /**
     *  @brief constructor to initialize member variables and also contruct the tree
     *
     *  @param points points to be stored in the trees
     *  @param maxLevel max level of tree nodes
     *  @param maxNumPoints max number of points to be stored in each node
     *  @param offsetMinPoint offset min point to control the boundary of the tree space
     *  @param offsetMaxPoint offset max point to control the boundary of the tree space
     */
    QuadOctreeBase(const VecPointType& points, uint8_t maxLevel, uint32_t maxNumPoints, const PointType& offsetMinPoint,
                   const PointType& offsetMaxPoint, const std::string& nodeName = "QuadOctreeBaseNode");

    /**
     *  @brief destructor to delete the root node
     *
     */
    ~QuadOctreeBase();

    /**
     *  @brief constant getter of the points member variable
     *
     */
    const VecPointType& points() const;

    /**
     *  @brief inorder traversal of the tree from root node
     *  @return coordinates about points stored in each node and center point of the node itself
     */
    virtual std::stringstream traversal() const;

    /**
     *  @brief check if a point is in the tree's boundary space
     *
     *  @param query the query point
     *  @return true if the point lies in the boudary space; false the otherwise
     */
    virtual bool insideSpace(const PointType& query) const;

    /**
     *  @brief find one neighbor point of a point (knn with k=1)
     *
     *  @param query the query point
     *  @param minDistance min distance from the query point to the neighbor point
     *  @return index of the neighbor point
     */
    virtual uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance = -1) const;

    /**
     *  @brief find k neighbors of a point
     *
     *  @param query the query point
     *  @param k number of neighbors to find
     *  @param minDistance min distance from the query point to the neighbor point
     *  @return vector of indices of the neighbors
     */
    virtual std::vector<uint32_t> knn(const PointType& query, uint32_t k, DATA_TYPE minDistance = -1) const;

 private:
    /**
     *  @brief struct of each node stored in the tree
     *
     */
    struct QuadOctreeBaseNode {
        /**
         *  @brief constructor with default value
         *
         */
        QuadOctreeBaseNode();

        /**
         *  @brief destructor to delete pointers to child nodes
         *
         */
        ~QuadOctreeBaseNode();

        //! flag to check if the current node is leaf or not
        bool isLeaf;

        //! coordinate values of the center point of the node space
        PointCoordinates coordinates;

        //! "square" radius of the node space
        DATA_TYPE radius;

        //! start index of the points stored in this node
        uint32_t startIdx;

        //! end index of the points stored in this node
        uint32_t endIdx;

        //! number of points stored in this node
        uint32_t numPoints;

        //! level of this node
        uint8_t level;

        //! pointers to child nodes
        QuadOctreeBaseNode* child[NUM_CHILD];
    };

    //! struct to represent the bounding box with coordinates of the center points and "square" radius
    struct BoundingBox {
        //! coordinates of the center point
        PointCoordinates coordinates;

        //! "square" radius
        DATA_TYPE radius;
    };

    /**
     *  @brief create a new tree node
     *
     *  @param coordinates coordinates of the center point of the node
     *  @param radius "square" radius
     *  @param startIdx start index of the points stored in this node
     *  @param endIdx end index of the points stored in this node
     *  @param numPoints number of points stored in this node
     *  @param level level of this node
     *  @return pointer to the new generated node
     */
    QuadOctreeBaseNode* createQuadOctreeBaseNode(const PointCoordinates& coordinates, DATA_TYPE radius,
                                                 uint32_t startIdx, uint32_t endIdx, uint32_t numPoints, uint8_t level);

    /**
     *  @brief block copy constructor
     *
     */
    QuadOctreeBase(const QuadOctreeBase&);

    /**
     *  @brief block assignment operator
     *
     */
    QuadOctreeBase& operator=(const QuadOctreeBase&);

    /**
     *  @brief check if a circle (quadtree case) or a ball (octree case) S(query, r) overlaps with the node space
     *
     *  @param query the query point
     *  @param radius the radius of the circle (or ball)
     *  @param quadOctreeBaseNode pointer to constant node
     *  @return true if overlapped; false the otherwise
     */
    bool overlaps(const PointType& query, DATA_TYPE radius, const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    /**
     *  @brief check if a circle (quadtree case) or a ball (octree case) S(query, r) is inside the node space
     *
     *  @param query the query point
     *  @param radius the radius of the circle (or ball)
     *  @param quadOctreeBaseNode pointer to constant node
     *  @return true if inside; false the otherwise
     */
    bool inside(const PointType& query, DATA_TYPE radius, const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    /**
     *  @brief check if a point lies inside the node space
     *
     *  @param query the query point
     *  @param radius the radius of the circle (or ball)
     *  @param quadOctreeBaseNode pointer to constant node
     *  @return true if inside; false the otherwise
     */
    bool inside(const PointType& query, const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    /**
     *  @brief estimate the boundary that cover all the input points
     *
     *  @param points input vector of points
     *  @param offsetMinPoint offset min point to control the size of the boundary
     *  @param offsetMaxPoint offset max point to control the size of the boundary
     *  @return bounding box of the boundary space
     */
    const BoundingBox estimateBounds(const VecPointType& points, const PointType& offsetMinPoint,
                                     const PointType& offsetMaxPoint) const;

    /**
     *  @brief find one neighbor point of a point (knn with k=1) starting from the target node
     *
     *  @param query the query point
     *  @param minDistance min distance from the query point to the neighbor point
     *  @param quadOctreeBaseNode pointer to constant node
     *  @return index of the neighbor point
     */
    uint32_t findNeighbor(const PointType& query, DATA_TYPE minDistance,
                          const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    /**
     *  @brief find k neighbors of a point starting from the target node
     *
     *  @param query the query point
     *  @param k number of neighbors to find
     *  @param minDistance min distance from the query point to the neighbor point
     *  @param quadOctreeBaseNode pointer to constant node
     *  @return vector of indices of the neighbors
     */
    std::vector<uint32_t> knn(const PointType& query, uint32_t k, DATA_TYPE minDistance,
                              const QuadOctreeBaseNode* quadOctreeBaseNode) const;

    /**
     *  @brief inorder traversal of the tree starting from the target node
     *  @param quadOctreeBaseNode pointer to constant node
     *  @return coordinates about points stored in each node and center point of the node itself
     */
    std::stringstream traversal(const QuadOctreeBaseNode* quadOctreeBaseNode) const;

 protected:
    //! vector of points held inside the tree
    VecPointType _points;

    //! pointer to root node
    QuadOctreeBaseNode* _root;

    //! vector to store the successive index of the point that follows the index of the current point.
    //! Supposed we have the vector of n points (p0,p1,...,p_(n-1)) with the index (i0=0;i1=1;...;i(n-1)=n-1).
    //! Then _successors[i_(j)] = i_(j+1) for j = [0,(n-1)].
    //! When a new permutation of n points is needed; we can keep the vector of points intact; and modify
    //! this successors vector
    std::vector<uint32_t> _successors;

    //! max level of a node
    uint8_t _maxLevel;

    //! max number of points stored in a node
    uint32_t _maxNumPoints;

    //! the name of node struct (quadrant for quadtree, octant for octree)
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

    this->_successors.resize(N);
    std::iota(this->_successors.begin(), this->_successors.end(), 1);

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
