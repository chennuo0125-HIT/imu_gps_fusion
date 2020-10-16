#include <Eigen/Core>
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
Eigen::Matrix<scalar, 3, 3> getRotFromAngleAxis(Eigen::Matrix<scalar, 3, 1> vec)
{
    scalar theta = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    Eigen::Matrix<scalar, 3, 1> unit_axis = vec / theta;
    Eigen::Matrix<scalar, 3, 3> unit_mat = Eigen::Matrix<scalar, 3, 3>::Identity();
    Eigen::Matrix<scalar, 3, 3> R;
    scalar sin_th = sin(theta);
    scalar cos_th = cos(theta);
    R = unit_mat * cos_th + (1 - cos_th) * unit_axis * unit_axis.transpose() + sin_th * vectorToSkewSymmetric<scalar>(unit_axis);

    return R;
}

} // namespace Fusion