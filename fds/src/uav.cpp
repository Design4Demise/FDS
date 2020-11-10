#include "../include/uav.h"
#include "../include/utils.h"

UAVClass::UAVClass() {

    utils::create_directories();

    read_parameters();
    calculate_gamma();

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

}

void UAVClass::write_state() {
    //
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
