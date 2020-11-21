#include "../include/uav.h"
#include "../include/utils.h"

UAVClass::UAVClass() {

    utils::create_directories();

    read_parameters();
    read_state();
    
    calculate_gamma();

    controlState = Eigen::Matrix<double, 4, 1>::Zero();

}

void UAVClass::read_parameters() {

    // read file
    std::ifstream ifs ("input/parameters.json");
    nlohmann::json json_file;
    
    ifs >> json_file;
    ifs.close();

    // load physical parameters
    mass = json_file["physical_parameters"]["mass"];

    Jx = json_file["physical_parameters"]["Jx"];
    Jy = json_file["physical_parameters"]["Jy"];
    Jz = json_file["physical_parameters"]["Jz"];
    Jxz = json_file["physical_parameters"]["Jxz"];

    S_wing = json_file["physical_parameters"]["S_wing"];
    b = json_file["physical_parameters"]["b"];
    c = json_file["physical_parameters"]["c"];
    e = json_file["physical_parameters"]["e"];
    AR = SQ(b) / S_wing;

    // load blending parameters
    sigma_m = json_file["blending_parameters"]["M"];
    alpha0 = json_file["blending_parameters"]["alpha0"];

    // load longitudinal parameters
    C_L_0 = json_file["longitudinal_parameters"]["C_L_0"];
    C_D_0 = json_file["longitudinal_parameters"]["C_D_0"];
    C_M_0 = json_file["longitudinal_parameters"]["C_M_0"];

    C_L_alpha = json_file["longitudinal_parameters"]["C_L_alpha"];
    C_D_alpha = json_file["longitudinal_parameters"]["C_D_alpha"];
    C_M_alpha = json_file["longitudinal_parameters"]["C_M_alpha"];

    C_L_q = json_file["longitudinal_parameters"]["C_L_q"];
    C_D_q = json_file["longitudinal_parameters"]["C_D_q"];
    C_M_q = json_file["longitudinal_parameters"]["C_M_q"];

    C_L_delta_e = json_file["longitudinal_parameters"]["C_L_delta_e"];
    C_D_delta_e = json_file["longitudinal_parameters"]["C_D_delta_e"];
    C_M_delta_e = json_file["longitudinal_parameters"]["C_M_delta_e"];

    // load lateral parameters
    C_Y_0 = json_file["lateral_parameters"]["C_Y_0"];
    C_ELL_0 = json_file["lateral_parameters"]["C_ELL_0"];
    C_N_0 = json_file["lateral_parameters"]["C_N_0"];

    C_Y_beta = json_file["lateral_parameters"]["C_Y_beta"];
    C_ELL_beta = json_file["lateral_parameters"]["C_ELL_beta"];
    C_N_beta = json_file["lateral_parameters"]["C_N_beta"];

    C_Y_p = json_file["lateral_parameters"]["C_Y_p"];
    C_ELL_p = json_file["lateral_parameters"]["C_ELL_p"];
    C_N_p = json_file["lateral_parameters"]["C_N_p"];

    C_Y_r = json_file["lateral_parameters"]["C_Y_r"];
    C_ELL_r = json_file["lateral_parameters"]["C_ELL_r"];
    C_N_r = json_file["lateral_parameters"]["C_N_r"];

    C_Y_delta_a = json_file["lateral_parameters"]["C_Y_delta_a"];
    C_ELL_delta_a = json_file["lateral_parameters"]["C_ELL_delta_a"];
    C_N_delta_a = json_file["lateral_parameters"]["C_N_delta_a"];

    C_Y_delta_r = json_file["lateral_parameters"]["C_Y_delta_r"];
    C_ELL_delta_r = json_file["lateral_parameters"]["C_ELL_delta_r"];
    C_N_delta_r = json_file["lateral_parameters"]["C_N_delta_r"];

}

void UAVClass::read_state() {

    // read file
    std::ifstream ifs ("input/state.json");
    nlohmann::json json_file;
    
    ifs >> json_file;
    ifs.close();

    state[es_px] = json_file["px0"];
    state[es_py] = json_file["py0"];
    state[es_pz] = json_file["pz0"];

    state[es_u] = json_file["u0"];
    state[es_v] = json_file["v0"];
    state[es_w] = json_file["w0"];

    std::array<double, 4> quat = utils::Euler2Quaternion(json_file["phi0"], json_file["theta0"], json_file["psi0"]);
    state[es_e0] = quat[e_e0];
    state[es_e1] = quat[e_e1];
    state[es_e2] = quat[e_e2];
    state[es_e3] = quat[e_e3];

    state[es_p] = json_file["p0"];
    state[es_q] = json_file["q0"];
    state[es_r] = json_file["r0"];

}

void UAVClass::write_state() {

    std::ofstream output;
    output.precision(PRECISION);

    if (!boost::filesystem::exists("Results/state.csv")) {
        output.open("Results/state.csv", std::ios::out);
        output << "t_step, time, px, py, pz, u, v, w, e0, e1, e2, e3, p, q, r" << std::endl;
    } else {
        output.open("Results/state.csv", std::ios::out | std::ios::app);
    }

    output << dynamicsPtr->curr_time_step << ", ";
    output << dynamicsPtr->curr_time_step * time_step << ", ";

    for (double s : state)
        output << s << ", ";

    output << std::endl;

    output.close();

}

void UAVClass::write_dstate() {

    std::ofstream output;
    output.precision(PRECISION);

    if (!boost::filesystem::exists("Results/dstate.csv")) {
        output.open("Results/dstate.csv", std::ios::out);
        output << "t_step, time, dpx, dpy, dpz, du, dv, dw, de0, de1, de2, de3, dp, dq, dr" << std::endl;
    } else {
        output.open("Results/dstate.csv", std::ios::out | std::ios::app);
    }

    output << dynamicsPtr->curr_time_step << ", ";
    output << dynamicsPtr->curr_time_step * time_step << ", ";
    
    for (double ds : dstate)
        output << ds << ", ";

    output << std::endl;

    output.close();

}

void UAVClass::calculate_gamma() {

    gamma = Jx * Jz - SQ(Jxz);

    gamma1 = (Jxz * (Jx - Jy + Jz)) / gamma;
    gamma2 = (Jz * (Jz - Jy) + SQ(Jxz)) / gamma;
    gamma3 = Jz / gamma;
    gamma4 = Jxz / gamma;
    gamma5 = (Jz - Jx) / Jy;
    gamma6 = Jxz / Jy;
    gamma7 = ((Jx - Jy) * Jx + SQ(Jxz)) / gamma;
    gamma8 = Jx / gamma;

}
