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

#ifndef OCTREE_HPP_
#define OCTREE_HPP_

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
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
template <typename T, class Container = Eigen::Matrix<T, 3, 1>>
class Octree : public std::enable_shared_from_this<Octree<T, Container>>
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = Container;
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    using Ptr = std::shared_ptr<Octree>;
    struct BoundingBox3D;

    Octree(const BoundingBox3D &bbox, int capacity, int depth);
    Octree(const BoundingBox3D &bbox, int capacity);
    explicit Octree(const BoundingBox3D &bbox);

    void subdivide();
    void insertElem(const PointType &point);
    void insertElems(const VecPointType &points);
    Octree::Ptr searchElem(const PointType &point);
    void deleteElem(const PointType &point);
    void deleteElems(const VecPointType &points);
    VecPointType queryRange(const BoundingBox3D &bbox);
    VecPointType nearestSearch(const PointType &point, size_t k);

    size_t indexOctantContainsPoint(const PointType &point);

    const BoundingBox3D &bbox() const
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
        return this->countElemsOfChild(std::enable_shared_from_this<Octree>::shared_from_this());
    }

 private:
    int countElemsOfChild(Octree::Ptr parentNode);
    void mergeIntoParent(Octree::Ptr parentNode);

 private:
    VecPointType _points;
    BoundingBox3D _bbox;
    int _capacity;
    int _depth;

    /*
        child:  0 1 2 3 4 5 6 7
        x:      - - - - + + + +
        y:      - - + + - - + +
        z:      - + - + - + - +
     */
    std::array<Octree::Ptr, 8> _child;
    Octree::Ptr _parent;
};

template <typename T, class Container>
Octree<T, Container>::Octree(const BoundingBox3D &bbox, int capacity, int depth)
    : _bbox(bbox), _capacity(capacity), _depth(depth)
{
    _points.reserve(capacity);
    std::fill(_child.begin(), _child.end(), nullptr);
    _parent = nullptr;
}

template <typename T, class Container>
Octree<T, Container>::Octree(const BoundingBox3D &bbox, int capacity) : _bbox(bbox), _capacity(capacity), _depth(0)
{
}

template <typename T, class Container> Octree<T, Container>::Octree(const BoundingBox3D &bbox) : Octree(bbox, 1)
{
}

template <typename T, class Container>
typename Octree<T, Container>::VecPointType Octree<T, Container>::queryRange(const BoundingBox3D &bbox)
{
    VecPointType results;

    if (!this->_bbox.intersectsBoundingBox(bbox)) {
        return results;
    }

    std::copy_if(this->_points.begin(), this->_points.end(), std::back_inserter(results),
                 [bbox](const PointType &p) { return bbox.containsPoint(p); });

    if (!this->_child[0]) {
        return results;
    }

    for (const Octree::Ptr &oPtr : this->_child) {
        VecPointType oPtrQueries = oPtr->queryRange(bbox);
        std::copy(oPtrQueries.begin(), oPtrQueries.end(), std::back_inserter(results));
    }

    return results;
}

template <typename T, class Container>
typename Octree<T, Container>::VecPointType Octree<T, Container>::nearestSearch(const PointType &point, size_t k)
{
    assert(k != 0);

    using DistancePoint = std::pair<double, PointType>;
    using DistanceOctant = std::pair<double, Octree::Ptr>;

    std::vector<DistancePoint> pointPriority;
    std::priority_queue<DistanceOctant, std::vector<DistanceOctant>, std::greater<DistanceOctant>> octantMinHeap;

    VecPointType nearestPoints;
    nearestPoints.reserve(k);

    octantMinHeap.push(
        std::make_pair(this->bbox().distanceToPoint(point), std::enable_shared_from_this<Octree>::shared_from_this()));

    while (!octantMinHeap.empty()) {
        auto currentDistQuadPair = octantMinHeap.top();
        Octree::Ptr currentQuad = currentDistQuadPair.second;

        if (pointPriority.size() >= k) {
            std::sort(pointPriority.begin(), pointPriority.end(),
                      [](const DistancePoint &dp1, const DistancePoint &dp2) { return dp1.first < dp2.first; });

            if (pointPriority[k - 1].first < currentDistQuadPair.first) {
                break;
            }
        }

        if (currentQuad->_child[0]) {
            for (const Octree::Ptr &qPtr : currentQuad->_child) {
                octantMinHeap.push(std::make_pair(qPtr->bbox().distanceToPoint(point), qPtr));
            }
        } else {
            for (const PointType &p : currentQuad->points()) {
                pointPriority.emplace_back(std::make_pair((p - point).norm(), p));
            }
        }

        octantMinHeap.pop();
    }

    for (size_t i = 0; i < pointPriority.size() && i < k; ++i) {
        nearestPoints.emplace_back(pointPriority[i].second);
    }

    return nearestPoints;
}

template <typename T, class Container> size_t Octree<T, Container>::indexOctantContainsPoint(const PointType &point)
{
    size_t idx = 0;
    if (point[0] >= this->_bbox.xCenter()) {
        idx |= 4;
    }

    if (point[1] >= this->_bbox.yCenter()) {
        idx |= 2;
    }

    if (point[2] >= this->_bbox.zCenter()) {
        idx |= 1;
    }

    return idx;
}

template <typename T, class Container> void Octree<T, Container>::subdivide()
{
    T xCenter = _bbox.xCenter();
    T yCenter = _bbox.yCenter();
    T zCenter = _bbox.zCenter();

    int currentDepth = this->_depth + 1;

    const PointType halfDimension = this->_bbox.halfDimension();

    for (size_t i = 0; i < 8; ++i) {
        {
            PointType newCentroid = this->_bbox.centroid();
            newCentroid[0] += halfDimension[0] * (i & 4 ? .5f : -.5f);
            newCentroid[1] += halfDimension[1] * (i & 2 ? .5f : -.5f);
            newCentroid[2] += halfDimension[2] * (i & 1 ? .5f : -.5f);

            this->_child[i] = std::make_shared<Octree>(BoundingBox3D(newCentroid, halfDimension * .5f), this->_capacity,
                                                       currentDepth);
            this->_child[i]->_parent = std::enable_shared_from_this<Octree>::shared_from_this();
        }
    }

    while (!this->_points.empty()) {
        const PointType &point = this->_points.back();
        this->_points.pop_back();
        this->_child[this->indexOctantContainsPoint(point)]->insertElem(point);
    }
}

template <typename T, class Container> void Octree<T, Container>::insertElem(const PointType &point)
{
    if (!this->_bbox.containsPoint(point)) {
        return;
    }

    // do not add duplicate
    if (std::find(this->_points.begin(), this->_points.end(), point) != this->_points.end()) {
        return;
    }

    // current node is leaf
    if (!this->_child[0]) {
        if (this->_points.size() < this->_capacity) {
            this->_points.emplace_back(point);
            return;
        }

        // the capacity is full now
        this->subdivide();
    }

    this->_child[this->indexOctantContainsPoint(point)]->insertElem(point);
}

template <typename T, class Container> void Octree<T, Container>::insertElems(const VecPointType &points)
{
    for (const PointType &point : points) {
        this->insertElem(point);
    }
}

template <typename T, class Container>
typename Octree<T, Container>::Ptr Octree<T, Container>::searchElem(const PointType &point)
{
    if (!this->_bbox.containsPoint(point)) {
        return nullptr;
    }

    // leaf node now
    if (!this->_child[0]) {
        if (std::find(this->_points.begin(), this->_points.end(), point) != this->_points.end()) {
            return std::enable_shared_from_this<Octree>::shared_from_this();
        }

        return nullptr;
    }

    return this->_child[this->indexOctantContainsPoint(point)]->searchElem(point);
}

template <typename T, class Container> void Octree<T, Container>::deleteElem(const PointType &point)
{
    Octree::Ptr ocHoldsPoint = this->searchElem(point);
    if (!ocHoldsPoint) {
        return;
    }

    ocHoldsPoint->points().erase(std::find(ocHoldsPoint->points().begin(), ocHoldsPoint->points().end(), point));

    Octree::Ptr ocParent = ocHoldsPoint->_parent;
    while (ocParent != nullptr && this->countElemsOfChild(ocParent) <= ocParent->_capacity) {
        this->mergeIntoParent(ocParent);
        ocParent = ocParent->_parent;
    }
}

template <typename T, class Container> void Octree<T, Container>::deleteElems(const VecPointType &points)
{
    for (const PointType &point : points) {
        this->deleteElem(point);
    }
}

template <typename T, class Container> int Octree<T, Container>::countElemsOfChild(Octree::Ptr parentNode)
{
    if (!parentNode) {
        return 0;
    }

    int result = parentNode->points().size();

    if (parentNode->_child[0]) {
        for (const Octree::Ptr &qPtr : parentNode->_child) {
            result += countElemsOfChild(qPtr);
        }
    }

    return result;
}

template <typename T, class Container> void Octree<T, Container>::mergeIntoParent(Octree<T, Container>::Ptr parentNode)
{
    if (!parentNode || !parentNode->_child[0]) {
        return;
    }

    for (Octree::Ptr &oPtr : parentNode->_child) {
        this->mergeIntoParent(oPtr);
        std::copy(oPtr->points().begin(), oPtr->points().end(), std::back_inserter(parentNode->points()));
        oPtr.reset();
        oPtr = nullptr;
    }
}

template <typename T, class Container> struct Octree<T, Container>::BoundingBox3D {
    T _xMin;
    T _yMin;
    T _zMin;

    T _xMax;
    T _yMax;
    T _zMax;

    BoundingBox3D(T xMin, T yMin, T zMin, T xMax, T yMax, T zMax)
        : _xMin(xMin), _yMin(yMin), _zMin(zMin), _xMax(xMax), _yMax(yMax), _zMax(zMax)
    {
    }

    BoundingBox3D(const PointType &centroid, const PointType &halfDimension)
    {
        _xMin = centroid[0] - halfDimension[0];
        _yMin = centroid[1] - halfDimension[1];
        _zMin = centroid[2] - halfDimension[2];

        _xMax = centroid[0] + halfDimension[0];
        _yMax = centroid[1] + halfDimension[1];
        _zMax = centroid[2] + halfDimension[2];
    }

    T xCenter()
    {
        return (_xMin + _xMax) / 2.0;
    }

    T yCenter()
    {
        return (_yMin + _yMax) / 2.0;
    }

    T zCenter()
    {
        return (_zMin + _zMax) / 2.0;
    }

    PointType centroid()
    {
        return PointType{xCenter(), yCenter(), zCenter()};
    }

    PointType halfDimension()
    {
        return PointType{(_xMax - _xMin) / 2, (_yMax - _yMin) / 2, (_zMax - _zMin) / 2};
    }

    bool containsPoint(const PointType &point) const
    {
        return _xMin <= point[0] && _yMin <= point[1] && _zMin <= point[2] && _xMax >= point[0] && _yMax >= point[1] &&
               _zMax >= point[2];
    }

    bool intersectsBoundingBox(const BoundingBox3D &other) const
    {
        return (this->_xMin <= other._xMax && this->_xMax >= other._xMin) &&
               (this->_yMin <= other._yMax && this->_yMax >= other._yMin) &&
               (this->_zMin <= other._zMax && this->_zMax >= other._zMin);
    }

    friend bool intersectsBoundBox(const BoundingBox3D &first, const BoundingBox3D &second)
    {
        return first.intersectsBoundingBox(second);
    }

    double distanceToPoint(const PointType &point) const
    {
        if (this->containsPoint(point)) {
            return 0;
        } else  // outside the cube
        {
            if (point[0] <= this->_xMax && point[0] >= this->_xMin) {
                if (point[1] <= this->_yMax && point[1] >= this->_yMin) {
                    return point[2] > this->_zMax ? (point[2] - this->_zMax) : (this->_zMin - point[2]);
                } else  // out of the range of y
                {
                    if (point[2] <= this->_zMax && point[2] >= this->_zMin) {
                        return point[1] > this->_yMax ? (point[1] - this->_yMax) : (this->_yMin - point[1]);
                    } else  // outside the range of y and z
                    {
                        T yAxis = point[1] > this->_yMax ? this->_yMax : this->_yMin;
                        T zAxis = point[2] > this->_zMax ? this->_zMax : this->_zMin;
                        return (point - PointType{point[0], yAxis, zAxis}).norm();
                    }
                }
            } else  // out of the range of x
            {
                if (point[1] <= this->_yMax && point[1] >= this->_yMin) {
                    if (point[2] <= this->_zMax && point[2] >= this->_zMin) {
                        return point[0] > this->_xMax ? (point[0] - this->_xMax) : (this->_xMin - point[0]);
                    } else  // out of the range of z  and x
                    {
                        T xAxis = point[0] > this->_xMax ? this->_xMax : this->_xMin;
                        T zAxis = point[2] > this->_zMax ? this->_zMax : this->_zMin;
                        return (point - PointType{xAxis, point[1], zAxis}).norm();
                    }
                } else  // out of the range of y and x
                {
                    if (point[2] <= this->_zMax && point[2] >= this->_zMin) {
                        T xAxis = point[0] > this->_xMax ? this->_xMax : this->_xMin;
                        T yAxis = point[1] > this->_yMax ? this->_yMax : this->_yMin;
                        return (point - PointType{xAxis, yAxis, point[2]}).norm();
                    } else  // out of the range of x, y and z
                    {
                        T xAxis = point[0] > this->_xMax ? this->_xMax : this->_xMin;
                        T yAxis = point[1] > this->_yMax ? this->_yMax : this->_yMin;
                        T zAxis = point[2] > this->_zMax ? this->_zMax : this->_zMin;
                        return (point - PointType{xAxis, yAxis, zAxis}).norm();
                    }
                }
            }
        }
    }

    friend std::ostream &operator<<(std::ostream &os, const BoundingBox3D &bbox)
    {
        os << "3d bounding box: \n";
        os << "xMin: " << bbox._xMin << "\n"
           << "yMin: " << bbox._yMin << "\n"
           << "zMin: " << bbox._zMin << "\n"
           << "xMax: " << bbox._xMax << "\n"
           << "yMax: " << bbox._yMax << "\n"
           << "zMax: " << bbox._zMax << "\n";

        return os;
    }
};

}  // namespace algo
#endif /* OCTREE_HPP_ */
