#include "../include/utils.h"

std::array<double, 3> utils::Quaternion2Euler(std::array<double, 4> q) {

    double e0 = q[e_e0];
    double e1 = q[e_e1];
    double e2 = q[e_e2];
    double e3 = q[e_e3];

    std::array<double, 3> euler;

    euler[e_phi] = std::atan2(2.0 * (e0 * e1 + e2 * e3), SQ(e0) + SQ(e3) - SQ(e1) - SQ(e2));
    euler[e_theta] = std::asin(2.0 * (e0 * e2 - e1 * e3));
    euler[e_psi] = std::atan2(2.0 * (e0 * e3 + e1 * e2), SQ(e0) + SQ(e1) - SQ(e2) - SQ(e3));

    return euler;
}

Eigen::Matrix3d utils::Quaternion2Rotation(std::array<double, 4> q) {

    double e0 = q[e_e0];
    double e1 = q[e_e1];
    double e2 = q[e_e2];
    double e3 = q[e_e3];

    Eigen::Matrix3d R;

    R(0, 0) = SQ(e1) + SQ(e0) - SQ(e2) - SQ(e3);
    R(0, 1) = 2.0 * (e1 * e2 - e3 * e0);
    R(0, 2) = 2.0 * (e1 * e3 + e2 * e0);

    R(1, 0) = 2.0 * (e1 * e2 + e3 * e0);
    R(1, 1) = SQ(e2) + SQ(e0) - SQ(e1) - SQ(e3);
    R(1, 2) = 2.0 * (e2 * e3 - e1 * e0);

    R(2, 0) = 2.0 * (e1 * e3 - e2 * e0);
    R(2, 1) = 2.0 * (e2 * e3 + e1 * e0);
    R(2, 2) = SQ(e3) + SQ(e0) - SQ(e1) - SQ(e2);

    R /= R.determinant();

    return R;
}

std::array<double, 4> utils::Euler2Quaternion(double phi, double theta, double psi) {

    std::array<double, 4> quaternion;

    quaternion[e_e0] = std::cos(psi / 2.0) * std::cos(theta / 2.0) * std::cos(phi / 2.0)
                       + std::sin(psi / 2.0) * std::sin(theta / 2.0) * std::sin(phi / 2.0);

    quaternion[e_e1] = std::cos(psi / 2.0) * std::cos(theta/ 2.0) * std::sin(phi / 2.0)
                       - std::sin(psi / 2.0) * std::sin(theta / 2.0) * std::cos(phi / 2.0);

    quaternion[e_e2] = std::cos(psi / 2.0) * std::sin(theta / 2.0) * std::cos(phi / 2.0)
                       + std::sin(psi / 2.0) * std::cos(theta / 2.0) * std::sin(phi / 2.0);

    quaternion[e_e3] = std::sin(psi / 2.0) * std::cos(theta / 2.0) * std::cos(phi / 2.0)
                       - std::cos(psi / 2.0) * std::sin(theta / 2.0) * std::sin(phi / 2.0);

    return quaternion;
}

Eigen::Matrix3d utils::Euler2Rotation(double phi, double theta, double psi) {

    double c_phi = std::cos(phi);
    double s_phi = std::sin(phi);

    double c_theta = std::cos(theta);
    double s_theta = std::sin(theta);

    double c_psi = std::cos(psi);
    double s_psi = std::sin(psi);

    Eigen::Matrix3d R, RRoll, RPitch, RYaw;

    RRoll << 1, 0, 0,
             0, c_phi, -s_phi,
             0, s_phi, c_phi;

    RPitch << c_theta, 0, s_theta,
              0, 1, 0,
              -s_theta, 0, c_theta;

    RYaw << c_psi, -s_psi, 0,
            s_psi, c_psi, 0,
            0, 0, 1;

    R = RYaw * (RPitch * RRoll);

    return R;
}







