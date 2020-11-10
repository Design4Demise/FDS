#include "../include/dynamics.h"
#include "../include/utils.h"

DynamicsClass::DynamicsClass() {

    std::cout << "Please initialise DynamicsClass with UAVClass object." << std::endl;
    std::exit(EXIT_FAILURE);

}

DynamicsClass::DynamicsClass(UAVClass &uav) {

    uavPtr = &uav;
    uavPtr->dynamicsPtr = this;

    curr_time_step = 0;
    v_air = uavPtr->state[es_u];

}

void DynamicsClass::update() {
    
    Eigen::Matrix<double, 13, 1> k1, k2, k3, k4;
    double norm_quaternion;

    // RK4 integration scheme
    k1 = calc_derivatives(uavPtr->state);
    k2 = calc_derivatives(uavPtr->state + time_step / 2.0 * k1);
    k3 = calc_derivatives(uavPtr->state + time_step / 2.0 * k2);
    k4 = calc_derivatives(uavPtr->state + time_step * k3);
    uavPtr->state += time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

    // normalise quaternion
    norm_quaternion = std::sqrt(SQ(uavPtr->state[es_e0]) + SQ(uavPtr->state[es_e1]) + SQ(uavPtr->state[es_e2])+ SQ(uavPtr->state[es_e3]));
    uavPtr->state[es_e0] /= norm_quaternion;
    uavPtr->state[es_e1] /= norm_quaternion;
    uavPtr->state[es_e2] /= norm_quaternion;
    uavPtr->state[es_e3] /= norm_quaternion;

}

Eigen::Matrix<double, 13, 1> DynamicsClass::calc_derivatives(Eigen::Matrix<double, 13, 1> in_state) {
    //   
}

void DynamicsClass::calc_forces_moments() {
    
    std::array<double, 4> quat = {uavPtr->state[es_e0], uavPtr->state[es_e1], uavPtr->state[es_e2], uavPtr->state[es_e3]};
    Eigen::Matrix3d rot = utils::Quaternion2Rotation(quat);

    Eigen::Vector3d gravity_vector(0.0, 0.0, uavPtr->mass * g0);
    Eigen::Vector3d f_gravity = rot.transpose() * gravity_vector;

    double fx = f_gravity[e_fx];
    double fy = f_gravity[e_fy];
    double fz = f_gravity[e_fz];

    double qbar = 0.5 * rho * SQ(v_air);
    double c_alpha = std::cos(alpha);
    double s_alpha = std::sin(alpha);

    double p_ndim = uavPtr->state[es_p] * uavPtr->b / (2.0 * v_air);
    double q_ndim = uavPtr->state[es_q] * uavPtr->c / (2.0 * v_air);
    double r_ndim = uavPtr->state[es_r] * uavPtr->b / (2.0 * v_air);

    double bk1 = std::exp(-uavPtr->sigma_m * (alpha - uavPtr->alpha0));
    double bk2 = std::exp(uavPtr->sigma_m * (alpha - uavPtr->alpha0));
    double sigma = (1 + bk1 + bk2) / ((1 + bk1) * (1 + bk2));

    double cl = ((1 - sigma) * (uavPtr->C_L_0 + uavPtr->C_L_alpha * alpha) + sigma * 2.0 * std::copysign(1, alpha) * SQ(s_alpha) * c_alpha);
    double cd = uavPtr->C_D_0 + SQ(uavPtr->C_L_0 + uavPtr->C_L_alpha * alpha) / (M_PI * uavPtr->e * uavPtr->AR);

    double f_lift = qbar * uavPtr->S_wing * (cl + uavPtr->C_L_q * q_ndim);
    double f_drag = qbar * uavPtr->S_wing * (cd + uavPtr->C_D_q * q_ndim);

    // calculating forces
    fx = fx - c_alpha * f_drag + s_alpha * f_lift;
    fz = fz = s_alpha * f_drag - c_alpha * f_lift;

    fy = fy + qbar * uavPtr->S_wing * (
        uavPtr->C_Y_0
        + uavPtr->C_Y_beta * beta
        + uavPtr->C_Y_p * p_ndim
        + uavPtr->C_Y_r * r_ndim
    );

    // calculating moments
    double My = qbar * uavPtr->S_wing * uavPtr->c * (
        uavPtr->C_M_0
        + uavPtr->C_M_alpha * alpha
        + uavPtr->C_M_q * q_ndim
    );

    double Mx = qbar * uavPtr->S_wing * uavPtr->b * (
        uavPtr->C_ELL_0
        + uavPtr->C_ELL_beta * beta
        + uavPtr->C_ELL_p * p_ndim
        + uavPtr->C_ELL_r * r_ndim
    );

    double Mz = qbar * uavPtr->S_wing * uavPtr->b * (
        uavPtr->C_N_0
        + uavPtr->C_N_beta * beta
        + uavPtr->C_N_p * p_ndim
        + uavPtr->C_N_r * r_ndim
    );

    forces_moments = {fx, fy, fz, Mx, My, Mz};

}
