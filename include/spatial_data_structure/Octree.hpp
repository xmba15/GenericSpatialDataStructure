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
#include <cstdlib>
#include <memory>
#include <vector>

#ifdef WITH_DEBUG
#define ENABLE_DEBUG 1
#endif

namespace algo
{
template <typename T, size_t POINT_DIMENSION = 3, class Container = Eigen::Matrix<T, 3, 1>> class Octree
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = Container;
    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    using Ptr = std::shared_ptr<Octree>;

    Octree();
    explicit Octree(const VecPointType& points);

 private:
    VecPointType _points;
};

template <typename T, size_t POINT_DIMENSION, class Container> Octree<T, POINT_DIMENSION, Container>::Octree()
{
}

template <typename T, size_t POINT_DIMENSION, class Container>
Octree<T, POINT_DIMENSION, Container>::Octree(const VecPointType& points) : _points(points)
{
}

}  // namespace algo
#endif /* OCTREE_HPP_ */
