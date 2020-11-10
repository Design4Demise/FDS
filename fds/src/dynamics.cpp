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

    alpha = 0;
    beta = 0;

}

void DynamicsClass::update() {
    
    Eigen::Matrix<double, 13, 1> k1, k2, k3, k4;
    double norm_quaternion;

    calc_forces_moments();

    // RK4 integration scheme
    k1 = calc_derivatives(uavPtr->state);
    k2 = calc_derivatives(uavPtr->state + time_step / 2.0 * k1);
    k3 = calc_derivatives(uavPtr->state + time_step / 2.0 * k2);
    k4 = calc_derivatives(uavPtr->state + time_step * k3);

    uavPtr->dstate = (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
    uavPtr->state += time_step * uavPtr->dstate;

    // normalise quaternion
    norm_quaternion = std::sqrt(SQ(uavPtr->state[es_e0]) + SQ(uavPtr->state[es_e1]) + SQ(uavPtr->state[es_e2])+ SQ(uavPtr->state[es_e3]));
    uavPtr->state[es_e0] /= norm_quaternion;
    uavPtr->state[es_e1] /= norm_quaternion;
    uavPtr->state[es_e2] /= norm_quaternion;
    uavPtr->state[es_e3] /= norm_quaternion;

    if (below_groundplane()) {
        std::cout << "UAV below groundplane, exiting..." << std::endl;
        std::exit(EXIT_SUCCESS);
    }

}

Eigen::Matrix<double, 13, 1> DynamicsClass::calc_derivatives(Eigen::Matrix<double, 13, 1> in_state) {

    Eigen::Matrix<double, 13, 1> d_vector;
    
    // get state variables
    double u = in_state[es_u];
    double v = in_state[es_v];
    double w = in_state[es_w];

    double e0 = in_state[es_e0];
    double e1 = in_state[es_e1];
    double e2 = in_state[es_e2];
    double e3 = in_state[es_e3];

    double p = in_state[es_p];
    double q = in_state[es_q];
    double r = in_state[es_r];

    // extract forces / moments
    double fx = forces_moments[efm_fx];
    double fy = forces_moments[efm_fy];
    double fz = forces_moments[efm_fz];
    double L = forces_moments[efm_L];
    double M = forces_moments[efm_M];
    double N = forces_moments[efm_N];

    // rotation matrix
    std::array<double, 4> quat = {e0, e1, e2, e3};
    Eigen::Matrix3d rot = utils::Quaternion2Rotation(quat);

    // position_derivatives
    Eigen::Vector3d velocity_vector(u, v, w);
    Eigen::Vector3d p_dot = rot * velocity_vector;
    d_vector[es_px] = p_dot[ex];
    d_vector[es_py] = p_dot[ey];
    d_vector[es_pz] = p_dot[ez];

    // velocity derivatives
    d_vector[es_u] = r * v - q * w + fx / uavPtr->mass;
    d_vector[es_v] = p * w - r * u + fy / uavPtr->mass;
    d_vector[es_w] = q * u - p * v + fz / uavPtr->mass;

    // rotational kinematics
    d_vector[es_e0] = 0.5 * (-p * e1 - q * e2 - r * e3);
    d_vector[es_e1] = 0.5 * (p * e0 + r * e2 - q * e3);
    d_vector[es_e2] = 0.5 * (q * e0 - r * e1 + p * e3);
    d_vector[es_e3] = 0.5 * (r * e0 + q * e1 - p * e2);

    // rotational dynamics
    d_vector[es_p] = (uavPtr->gamma1 * p * q - uavPtr->gamma2 * q * r + uavPtr->gamma3 * L + uavPtr->gamma4 * N);
    d_vector[es_q] = (uavPtr->gamma5 * p * r - uavPtr->gamma6 * (SQ(p) - SQ(r)) + M / uavPtr->Jy);
    d_vector[es_r] = (uavPtr->gamma7 * p * q - uavPtr->gamma1 * q * r + uavPtr->gamma4 * L + uavPtr->gamma8 * N);

    return d_vector;
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
    fz = fz - s_alpha * f_drag - c_alpha * f_lift;

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

bool DynamicsClass::below_groundplane() {

    return uavPtr->state[es_pz] > 0.0;

}
