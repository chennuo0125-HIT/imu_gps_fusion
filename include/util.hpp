#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

namespace Fusion
{

template <typename scalar>
Eigen::Matrix<scalar, 3, 3> vectorToSkewSymmetric(Eigen::Matrix<scalar, 3, 1> vec)
{
    Eigen::Matrix<scalar, 3, 3> M;
    M << 0, -vec(2), vec(1),
        vec(2), 0, -vec(0),
        -vec(1), vec(0), 0;

    return M;
}

// get rotation matrix from angleaxis by rodrigues formula
template <typename scalar>
Eigen::Matrix<scalar, 3, 3> getRotFromAA(Eigen::Matrix<scalar, 3, 1> vec)
{
    scalar theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (0 == theta)
        return Eigen::Matrix<scalar, 3, 3>::Identity();
    Eigen::Matrix<scalar, 3, 1> unit_axis = vec / theta;
    Eigen::Matrix<scalar, 3, 3> unit_mat = Eigen::Matrix<scalar, 3, 3>::Identity();
    Eigen::Matrix<scalar, 3, 3> R;
    scalar sin_th = sin(theta);
    scalar cos_th = cos(theta);
    R = unit_mat * cos_th + (1 - cos_th) * unit_axis * unit_axis.transpose() + sin_th * vectorToSkewSymmetric<scalar>(unit_axis);

    return R;
}

// tranform angleaxis to quaternion
template <typename scalar>
Eigen::Quaternion<scalar> getQuaFromAA(Eigen::Matrix<scalar, 3, 1> vec)
{
    scalar theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    if (0 == theta)
        return Eigen::Quaternion<scalar>::Identity();
    Eigen::Matrix<scalar, 3, 1> unit_axis = vec / theta;
    scalar w = cos(0.5 * theta);
    scalar sin_t = sin(0.5 * theta);
    scalar x = unit_axis[0] * sin_t;
    scalar y = unit_axis[1] * sin_t;
    scalar z = unit_axis[2] * sin_t;
    Eigen::Quaternion<scalar> qua(w, x, y, z);
    qua.normalized();

    return qua;
}
} // namespace Fusion