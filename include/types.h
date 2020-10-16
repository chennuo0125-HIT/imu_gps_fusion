#ifndef _TYPES_H_
#define _TYPES_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Fusion
{

// imu data struct
template <typename scalar>
struct ImuData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<scalar, 3, 1> acc;
    Eigen::Matrix<scalar, 3, 1> gyr;
};

// gps data struct
template <typename scalar>
struct GpsData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<scalar, 3, 1> data;
    Eigen::Matrix<scalar, 3, 3> cov;
};

// formula: 156
template <typename scalar>
struct State
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<scalar, 3, 1> p;
    Eigen::Matrix<scalar, 3, 1> v;
    Eigen::Quaternion<scalar> q;
    Eigen::Matrix<scalar, 3, 1> a_b;
    Eigen::Matrix<scalar, 3, 1> w_b;
};

template <typename scalar>
struct ErrorState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<scalar, 3, 1> d_p;
    Eigen::Matrix<scalar, 3, 1> d_v;
    Eigen::Matrix<scalar, 3, 1> d_theta;
    Eigen::Matrix<scalar, 3, 1> d_a_b;
    Eigen::Matrix<scalar, 3, 1> d_w_b;
};

// formula: 158, 159, 160, 161
template <typename scalar>
struct ImuVar
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<scalar, 3, 3> Vi;
    Eigen::Matrix<scalar, 3, 3> Ti;
    Eigen::Matrix<scalar, 3, 3> Ai;
    Eigen::Matrix<scalar, 3, 3> Wi;
};

template <typename scalar>
struct GpsVar
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<scalar, 3, 3> Pi;
};

} // namespace Fusion

#endif