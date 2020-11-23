#ifndef UAV_H
#define UAV_H

#include "input.h"
#include "dynamics.h"
#include "propulsion.h"


// forward declarations
class DynamicsClass;
class PropulsionClass;


class UAVClass {

    // friend class
    friend class DynamicsClass;
    friend class PropulsionClass;

// ctor/dtor
public:

    UAVClass();
    ~UAVClass() {};

// public attributes
public:

    Eigen::Matrix<double, 13, 1> state;       // state vector 
    Eigen::Matrix<double, 13, 1> dstate;      // state vector derivatives

    Eigen::Matrix<double, 4, 1> controlState; // control vector

    double mass;                              // mass of the aircraft
    double Jx, Jy, Jz, Jxz;                   // moments of inertia

    double S_wing, AR, b, c, e;               // wing parameters
    double sigma_m, alpha0;                   // aerodynamic blending

    // longitudinal parameters
    double C_L_0, C_D_0, C_M_0;
    double C_L_alpha, C_D_alpha, C_M_alpha;
    double C_L_q, C_D_q, C_M_q;
    double C_L_delta_e, C_D_delta_e, C_M_delta_e;

    // lateral parameters
    double C_Y_0, C_ELL_0, C_N_0;
    double C_Y_beta, C_ELL_beta, C_N_beta;
    double C_Y_p, C_ELL_p, C_N_p;
    double C_Y_r, C_ELL_r, C_N_r;
    double C_Y_delta_a, C_ELL_delta_a, C_N_delta_a;
    double C_Y_delta_r, C_ELL_delta_r, C_N_delta_r;

    // prop parameters
    double D_prop;
    double K_V, KQ;

    double R_motor;
    double i0;
    double ncells, Vmax;

    double C_Q0, C_Q1, C_Q2;
    double C_T0, C_T1, C_T2;

    

// private attributes
private:

    // dynamics pointer
    DynamicsClass *dynamicsPtr;

    // propulsion pointer
    PropulsionClass *propulsionPtr;

    // calculation parameters
    double gamma, gamma1, gamma2, gamma3, gamma4, gamma5, gamma6, gamma7, gamma8;

// public methods
public:

    void read_parameters();
    void read_state();

    void write_state();
    void write_dstate();

private:

    void calculate_gamma();

};

#endif
