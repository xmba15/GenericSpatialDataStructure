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

#ifndef OCTREE_HPP_
#define OCTREE_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <ostream>
#include <queue>
#include <utility>
#include <vector>

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace algo
{
template <typename T, class Container = Eigen::Matrix<T, 2, 1>>
class Quadtree : public std::enable_shared_from_this<Quadtree<T, Container>>
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = Container;
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using Ptr = std::shared_ptr<Quadtree>;
    struct BoundingBox;

    Quadtree(const BoundingBox &bbox, int capacity, int depth);
    Quadtree(const BoundingBox &bbox, int capacity);
    explicit Quadtree(const BoundingBox &bbox);

    void subdivide();
    void insertElem(const PointType &point);
    void insertElems(const VecPointType &points);
    Quadtree::Ptr searchElem(const PointType &point);
    void deleteElem(const PointType &point);
    void deleteElems(const VecPointType &points);
    VecPointType queryRange(const BoundingBox &bbox);
    VecPointType nearestSearch(const PointType &point, size_t k);

    const BoundingBox &bbox() const
    {
        return this->_bbox;
    }

    const VecPointType &points() const
    {
        return this->_points;
    }

    VecPointType &points()
    {
        return this->_points;
    }

    int pointSize()
    {
        return this->countElemsOfChild(std::enable_shared_from_this<Quadtree>::shared_from_this());
    }

 private:
    void mergeIntoParent(Quadtree::Ptr parentNode);
    int countElemsOfChild(Quadtree::Ptr parentNode);

 private:
    VecPointType _points;
    BoundingBox _bbox;
    int _capacity;
    int _depth;

    Quadtree::Ptr _nw;
    Quadtree::Ptr _ne;
    Quadtree::Ptr _sw;
    Quadtree::Ptr _se;
    Quadtree::Ptr _parent;
};

template <typename T, class Container>
Quadtree<T, Container>::Quadtree(const BoundingBox &bbox, int capacity, int depth)
    : _bbox(bbox), _capacity(capacity), _depth(depth)
{
    _points.reserve(capacity);

    _nw = nullptr;
    _ne = nullptr;
    _sw = nullptr;
    _se = nullptr;
    _parent = nullptr;
}

template <typename T, class Container>
Quadtree<T, Container>::Quadtree(const BoundingBox &bbox, int capacity) : _bbox(bbox), _capacity(capacity), _depth(0)
{
}

template <typename T, class Container> Quadtree<T, Container>::Quadtree(const BoundingBox &bbox) : Quadtree(bbox, 1)
{
}

template <typename T, class Container> void Quadtree<T, Container>::subdivide()
{
    T xCenter = _bbox.xCenter();
    T yCenter = _bbox.yCenter();

    int currentDepth = this->_depth + 1;

    _nw = std::make_shared<Quadtree>(BoundingBox(_bbox._xMin, yCenter, xCenter, _bbox._yMax), this->_capacity,
                                     currentDepth);
    _ne = std::make_shared<Quadtree>(BoundingBox(xCenter, yCenter, _bbox._xMax, _bbox._yMax), this->_capacity,
                                     currentDepth);
    _sw = std::make_shared<Quadtree>(BoundingBox(_bbox._xMin, _bbox._yMin, xCenter, yCenter), this->_capacity,
                                     currentDepth);
    _se = std::make_shared<Quadtree>(BoundingBox(xCenter, _bbox._yMin, _bbox._xMax, yCenter), this->_capacity,
                                     currentDepth);

    _nw->_parent = std::enable_shared_from_this<Quadtree>::shared_from_this();
    _ne->_parent = std::enable_shared_from_this<Quadtree>::shared_from_this();
    _sw->_parent = std::enable_shared_from_this<Quadtree>::shared_from_this();
    _se->_parent = std::enable_shared_from_this<Quadtree>::shared_from_this();

    while (!this->_points.empty()) {
        const PointType &point = this->_points.back();
        this->_points.pop_back();
        this->_nw->insertElem(point);
        this->_ne->insertElem(point);
        this->_sw->insertElem(point);
        this->_se->insertElem(point);
    }
}

template <typename T, class Container> void Quadtree<T, Container>::insertElem(const PointType &point)
{
    if (!this->_bbox.containsPoint(point)) {
        return;
    }

    // do not add duplicate
    if (std::find(this->_points.begin(), this->_points.end(), point) != this->_points.end()) {
        return;
    }

    // current node is leaf
    if (!this->_nw) {
        if (this->_points.size() < this->_capacity) {
            this->_points.emplace_back(point);
            return;
        }

        // the capacity is full now
        this->subdivide();
    }

    this->_nw->insertElem(point);
    this->_ne->insertElem(point);
    this->_sw->insertElem(point);
    this->_se->insertElem(point);
}

template <typename T, class Container>
typename Quadtree<T, Container>::Ptr Quadtree<T, Container>::searchElem(const PointType &point)
{
    Quadtree::Ptr result = nullptr;

    if (!this->_bbox.containsPoint(point)) {
        return result;
    }

    // leaf node now
    if (!this->_nw) {
        if (std::find(this->_points.begin(), this->_points.end(), point) != this->_points.end()) {
            return std::enable_shared_from_this<Quadtree>::shared_from_this();
        }

        return result;
    }

    for (const Quadtree::Ptr &qPtr : {_nw, _ne, _sw, _se}) {
        if (qPtr->_bbox.containsPoint(point)) {
            result = qPtr->searchElem(point);
        }
    }

    return result;
}

template <typename T, class Container> void Quadtree<T, Container>::insertElems(const VecPointType &points)
{
    for (const PointType &point : points) {
        this->insertElem(point);
    }
}

template <typename T, class Container> void Quadtree<T, Container>::deleteElem(const PointType &point)
{
    Quadtree::Ptr quadHoldsPoint = this->searchElem(point);
    if (!quadHoldsPoint) {
        return;
    }

    quadHoldsPoint->points().erase(std::find(quadHoldsPoint->points().begin(), quadHoldsPoint->points().end(), point));

    Quadtree::Ptr quadParent = quadHoldsPoint->_parent;
    while (quadParent != nullptr && this->countElemsOfChild(quadParent) <= quadParent->_capacity) {
        this->mergeIntoParent(quadParent);
        quadParent = quadParent->_parent;
    }
}

template <typename T, class Container> void Quadtree<T, Container>::deleteElems(const VecPointType &points)
{
    for (const PointType &point : points) {
        this->deleteElem(point);
    }
}

template <typename T, class Container>
typename Quadtree<T, Container>::VecPointType Quadtree<T, Container>::queryRange(const BoundingBox &bbox)
{
    VecPointType results;

    if (!this->_bbox.intersectsBoundingBox(bbox)) {
        return results;
    }

    std::copy_if(this->_points.begin(), this->_points.end(), std::back_inserter(results),
                 [bbox](const PointType &p) { return bbox.containsPoint(p); });

    if (!this->_nw) {
        return results;
    }

    for (const Quadtree::Ptr &qPtr : {_nw, _ne, _sw, _se}) {
        VecPointType qPtrQueries = qPtr->queryRange(bbox);
        std::copy(qPtrQueries.begin(), qPtrQueries.end(), std::back_inserter(results));
    }

    return results;
}

template <typename T, class Container>
typename Quadtree<T, Container>::VecPointType Quadtree<T, Container>::nearestSearch(const PointType &point, size_t k)
{
    assert(k != 0);

    using DistancePoint = std::pair<double, PointType>;
    using DistanceQuadrant = std::pair<double, Quadtree::Ptr>;

    std::vector<DistancePoint> pointPriority;
    std::priority_queue<DistanceQuadrant, std::vector<DistanceQuadrant>, std::greater<DistanceQuadrant>>
        quadrantMinHeap;

    VecPointType nearestPoints;
    nearestPoints.reserve(k);

    quadrantMinHeap.push(std::make_pair(this->bbox().distanceToPoint(point),
                                        std::enable_shared_from_this<Quadtree>::shared_from_this()));

    while (!quadrantMinHeap.empty()) {
        auto currentDistQuadPair = quadrantMinHeap.top();
        Quadtree::Ptr currentQuad = currentDistQuadPair.second;

        if (pointPriority.size() >= k) {
            std::sort(pointPriority.begin(), pointPriority.end(),
                      [](const DistancePoint &dp1, const DistancePoint &dp2) { return dp1.first < dp2.first; });

            if (pointPriority[k - 1].first < currentDistQuadPair.first) {
                break;
            }
        }

        if (currentQuad->_nw) {
            for (const Quadtree::Ptr &qPtr : {currentQuad->_nw, currentQuad->_ne, currentQuad->_sw, currentQuad->_se}) {
                quadrantMinHeap.push(std::make_pair(qPtr->bbox().distanceToPoint(point), qPtr));
            }
        } else {
            for (const PointType &p : currentQuad->points()) {
                pointPriority.emplace_back(std::make_pair((p - point).norm(), p));
            }
        }

        quadrantMinHeap.pop();
    }

    for (size_t i = 0; i < pointPriority.size() && i < k; ++i) {
        nearestPoints.emplace_back(pointPriority[i].second);
    }

    return nearestPoints;
}

template <typename T, class Container> int Quadtree<T, Container>::countElemsOfChild(Quadtree::Ptr parentNode)
{
    if (!parentNode) {
        return 0;
    }

    int result = parentNode->points().size();

    if (parentNode->_nw) {
        for (const Quadtree::Ptr &qPtr : {parentNode->_nw, parentNode->_ne, parentNode->_sw, parentNode->_se}) {
            result += countElemsOfChild(qPtr);
        }
    }

    return result;
}

template <typename T, class Container> void Quadtree<T, Container>::mergeIntoParent(Quadtree::Ptr parentNode)
{
    if (!parentNode || !parentNode->_nw) {
        return;
    }

    for (Quadtree::Ptr qPtr : {parentNode->_nw, parentNode->_ne, parentNode->_sw, parentNode->_se}) {
        this->mergeIntoParent(qPtr);
        std::copy(qPtr->points().begin(), qPtr->points().end(), std::back_inserter(parentNode->points()));
        qPtr.reset();
        qPtr = nullptr;
    }
}

template <typename T, class Container> struct Quadtree<T, Container>::BoundingBox {
    T _xMin;
    T _yMin;
    T _xMax;
    T _yMax;

    BoundingBox(T xMin, T yMin, T xMax, T yMax) : _xMin(xMin), _yMin(yMin), _xMax(xMax), _yMax(yMax)
    {
    }

    bool containsPoint(const PointType &point) const
    {
        return _xMin <= point[0] && _yMin <= point[1] && _xMax >= point[0] && _yMax >= point[1];
    }

    /**
                    I   |    II    |  III
                  ======+==========+======   --yMax
                   VIII |  IX (in) |  IV
                  ======+==========+======   --yMin
                   VII  |    VI    |   V
                        '          '
                        '          '
                       xMin       xMax
    **/
    double distanceToPoint(const PointType &point) const
    {
        if (point[0] < this->_xMin)  // Region 1, 8, 7
        {
            if (point[1] > this->_yMax)  // Region 1
            {
                return (point - PointType{this->_xMin, this->_yMax}).norm();
            } else if (point[1] < this->_yMin)  // Region 7
            {
                return (point - PointType{this->_xMin, this->_yMin}).norm();
            } else  // Region 8
            {
                return this->_xMin - point[0];
            }
        } else if (point[0] > this->_xMax)  // Region 3,4,5
        {
            if (point[1] > this->_yMax)  // Region 3
            {
                return (point - PointType{this->_xMax, this->_yMax}).norm();
            } else if (point[1] < this->_yMin)  // Region 5
            {
                return (point - PointType{this->_xMax, this->_yMin}).norm();
            } else  // Region 4
            {
                return point[0] - this->_xMax;
            }
        } else  //  Region 2,9,6
        {
            if (point[1] > this->_yMax)  // Region 2
            {
                return point[1] - this->_yMax;
            } else if (point[1] < this->_yMin)  // Region 6
            {
                return this->_yMin - point[1];
            } else  // Region 9
            {
                return 0;
            }
        }
    }

    bool intersectsBoundingBox(const BoundingBox &other) const
    {
        return (this->_xMin <= other._xMax && this->_xMax >= other._xMin) &&
               (this->_yMin <= other._yMax && this->_yMax >= other._yMin);
    }

    friend bool intersectsBoundBox(const BoundingBox &first, const BoundingBox &second)
    {
        return first.intersectsBoundingBox(second);
    }

    T xCenter()
    {
        return (_xMin + _xMax) / 2.0;
    }

    T yCenter()
    {
        return (_yMin + _yMax) / 2.0;
    }

    friend std::ostream &operator<<(std::ostream &os, const BoundingBox &bbox)
    {
        os << "bounding box: \n";
        os << "xMin: " << bbox._xMin << "\n"
           << "yMin: " << bbox._yMin << "\n"
           << "xMax: " << bbox._xMax << "\n"
           << "yMax: " << bbox._yMax << "\n";

        return os;
    }
};

}  // namespace algo
#endif /* OCTREE_HPP_ */
