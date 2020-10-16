#include "imu_gps_fusion.h"
#include "util.hpp"

namespace Fusion
{

ImuGpsFusion::ImuGpsFusion()
{
    // initialize Fi
    Fi_.setZero();
    Fi_.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    // initialize Hx
    Hx_.setZero();
    Hx_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();

    // set init state
    no_state_.p.setZero();
    no_state_.q.setIdentity();
    no_state_.v.setZero();
    no_state_.a_b.setZero();
    no_state_.w_b.setZero();
    ac_state_ = no_state_;

    // init gravity
    g_ = Eigen::Vector3d(0.0, 0.0, -9.805);
}

ImuGpsFusion::~ImuGpsFusion() {}

void ImuGpsFusion::cfgImuVar(double sigma_an, double sigma_wn, double sigma_aw, double sigma_ww)
{
    sigma_an_2_ = sigma_an * sigma_an;
    sigma_wn_2_ = sigma_wn * sigma_wn;
    sigma_aw_2_ = sigma_aw * sigma_aw;
    sigma_ww_2_ = sigma_ww * sigma_ww;
}

void ImuGpsFusion::cfgRefGps(double latitude, double longitute, double altitude)
{
    ref_lati_ = latitude;
    ref_long_ = longitute;
    ref_alti_ = altitude;
}

void ImuGpsFusion::updateNominalState(const ImuData<double> &imu_data, double dt)
{
    Eigen::Matrix3d R(no_state_.q);
    no_state_.p = no_state_.p + no_state_.v * dt + 0.5 * (R * (imu_data.acc - no_state_.a_b) + g_) * dt * dt;
    no_state_.v = no_state_.v + (R * (imu_data.acc - no_state_.a_b) + g_) * dt;
    Eigen::Vector3d q_v = (imu_data.gyr - no_state_.w_b) * dt;
    no_state_.q = no_state_.q * Eigen::Quaterniond(1, q_v[0], q_v[1], q_v[2]);
}

void ImuGpsFusion::calcF(const ImuData<double> &imu_data, double dt)
{
    Fx_.setIdentity();
    Fx_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    Eigen::Matrix3d R(no_state_.q);
    Fx_.block<3, 3>(3, 6) = -R * vectorToSkewSymmetric<double>(imu_data.acc - no_state_.a_b) * dt;
    Fx_.block<3, 3>(3, 9) = -R * dt;
    Fx_.block<3, 3>(6, 6) = R.inverse() * getRotFromAngleAxis<double>((imu_data.gyr - no_state_.w_b) * dt);
    Fx_.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;
}

void ImuGpsFusion::updateQ(double dt)
{
    Qi_.setIdentity();
    Qi_.block<3, 3>(0, 0) *= sigma_an_2_ * dt * dt;
    Qi_.block<3, 3>(3, 3) *= sigma_wn_2_ * dt * dt;
    Qi_.block<3, 3>(6, 6) *= sigma_aw_2_ * dt;
    Qi_.block<3, 3>(9, 9) *= sigma_wn_2_ * dt;
}

void ImuGpsFusion::imuPredict(const ImuData<double> &imu_data, double dt)
{
    updateNominalState(imu_data, dt);
    calcF(imu_data, dt);
    updateQ(dt);
    P_ = Fx_ * P_ * Fx_.transpose() + Fi_ * Qi_ * Fi_.transpose();
}

void ImuGpsFusion::updateH()
{
    // calculate X_dx
    Eigen::Matrix<double, 16, 15> X_dx;
    X_dx.setZero();
    X_dx.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    X_dx.block<6, 6>(10, 9) = Eigen::Matrix<double, 6, 6>::Identity();
    X_dx.block<4, 3>(6, 6) << -no_state_.q.x(), -no_state_.q.y(), -no_state_.q.z(),
        no_state_.q.w(), -no_state_.q.z(), no_state_.q.y(),
        no_state_.q.z(), no_state_.q.w(), -no_state_.q.x(),
        -no_state_.q.y(), no_state_.q.x(), no_state_.q.w();
    X_dx = 0.5 * X_dx;

    // use the chain rule to update H matrix
    H_ = Hx_ * X_dx;
}

void ImuGpsFusion::updateV(const GpsData<double> &gps_data)
{
    V_ = gps_data.cov;
}

void ImuGpsFusion::gpsUpdate(const GpsData<double> &gps_data, const vector<pair<ImuData<double>, double>> &imu_datas)
{
    // predict stage
    for (const auto &iter : imu_datas)
    {
        imuPredict(iter.first, iter.second);
    }
    cout << "imuPredict: ";
    cout << no_state_.p[0] << " " << no_state_.p[1] << " " << no_state_.p[2] << " "
         << no_state_.q.w() << " " << no_state_.q.x() << " " << no_state_.q.y() << " " << no_state_.q.z() << endl;

    // transform gps latitude and longitude coordinate to position in enu frame
    double enu[3];
    double lla[3] = {gps_data.data[0], gps_data.data[1], gps_data.data[2]};
    double ref[3] = {ref_lati_, ref_long_, ref_alti_};
    gpsConverter_.lla2enu(enu, lla, ref);
    cout << "gps position: " << enu[0] << " " << enu[1] << " " << enu[2] << endl;

    // update
    updateH();
    Eigen::Matrix<double, 15, 3> K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + V_).inverse();
    Eigen::Matrix<double, 15, 1> dX = K * (Eigen::Vector3d(enu[0], enu[1], enu[2]) - no_state_.p);
    ErrorState<double> estate;
    estate.d_p = dX.block<3, 1>(0, 0);
    estate.d_v = dX.block<3, 1>(3, 0);
    estate.d_theta = dX.block<3, 1>(6, 0);
    estate.d_a_b = dX.block<3, 1>(9, 0);
    estate.d_w_b = dX.block<3, 1>(12, 0);
    P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H_) * P_;
    ac_state_.p = no_state_.p + estate.d_p;
    ac_state_.v = no_state_.v + estate.d_v;
    ac_state_.q = no_state_.q * Eigen::Quaterniond(1, estate.d_theta[0], estate.d_theta[1], estate.d_theta[2]);
    ac_state_.a_b = no_state_.a_b + estate.d_a_b;
    ac_state_.w_b = no_state_.w_b + estate.d_w_b;
    no_state_ = ac_state_;
    cout << "gpsUpdate: ";
    cout << no_state_.p[0] << " " << no_state_.p[1] << " " << no_state_.p[2] << " "
         << no_state_.q.w() << " " << no_state_.q.x() << " " << no_state_.q.y() << " " << no_state_.q.z() << endl;
}

State<double> ImuGpsFusion::getState()
{
    return ac_state_;
}

} // namespace Fusion