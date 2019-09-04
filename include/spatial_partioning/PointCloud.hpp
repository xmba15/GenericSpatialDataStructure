/**
 * @file    PointCloud.hpp
 *
 * @author  btran
 *
 * @date    2019-09-04
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <vector>

#include "spatial_partioning/Octree.hpp"

namespace algo
{
namespace geometry
{
template <typename DATA_TYPE, class PointContainer = Eigen::Matrix<DATA_TYPE, 3, 1>> class PointCloudBase
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using PointType = PointContainer;

    using VecPointType = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    PointCloudBase();

    explicit PointCloudBase(const VecPointType& points);

    ~PointCloudBase() = default;

    bool hasPoints() const
    {
        return points_.size() > 0;
    }

    bool hasNormals() const
    {
        return points_.size() > 0 && normals_.size() == points_.size();
    }

    bool hasColors() const
    {
        return points_.size() > 0 && colors_.size() == points_.size();
    }

    PointCloudBase& Clear()
    {
        points_.clear();
        normals_.clear();
        colors_.clear();
        return *this;
    }

    std::array<std::vector<DATA_TYPE>, 3> extractDataEachAxis(const VecPointType& vecPointType) const;

    std::array<std::vector<DATA_TYPE>, 3> extractPointDataEachAxis() const;

 private:
    PointCloudBase(const PointCloudBase&);
    PointCloudBase& operator=(const PointCloudBase&);

 public:
    VecPointType points_;
    VecPointType normals_;
    VecPointType colors_;
};

template <typename DATA_TYPE, class PointContainer> PointCloudBase<DATA_TYPE, PointContainer>::PointCloudBase()
{
}

template <typename DATA_TYPE, class PointContainer>
PointCloudBase<DATA_TYPE, PointContainer>::PointCloudBase(const VecPointType& points) : points_(points)
{
}

template <typename DATA_TYPE, class PointContainer>
std::array<std::vector<DATA_TYPE>, 3>
PointCloudBase<DATA_TYPE, PointContainer>::extractDataEachAxis(const VecPointType& vecPointType) const
{
    std::array<std::vector<DATA_TYPE>, 3> result;

    for (size_t i = 0; i < 3; ++i) {
        std::vector<DATA_TYPE> curV;
        curV.reserve(vecPointType.size());
        std::transform(vecPointType.begin(), vecPointType.end(), std::back_inserter(curV),
                       [&i](const PointType& p) { return p[i]; });
        result[i] = curV;
    }
    return result;
}

template <typename DATA_TYPE, class PointContainer>
std::array<std::vector<DATA_TYPE>, 3> PointCloudBase<DATA_TYPE, PointContainer>::extractPointDataEachAxis() const
{
    return this->extractDataEachAxis(this->points_);
}

using PointCloud = PointCloudBase<double, Eigen::Matrix<double, 3, 1>>;

}  // namespace geometry
}  // namespace algo
