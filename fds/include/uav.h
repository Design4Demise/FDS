#ifndef UAV_H
#define UAV_H

#include "input.h"
#include "dynamics.h"

// forward declaration
class DynamicsClass;

class UAVClass {

    // friend class
    friend class DynamicsClass;

// con/de-structors
public:

    UAVClass();
    ~UAVClass() {};

// public attributes
public:

    Eigen::Matrix<double, 13, 1> state;    // state vector 
    Eigen::Matrix<double, 13, 1> dstate;   // state vector derivatives

    double mass;                           // mass of the aircraft
    double Jx, Jy, Jz, Jxz;                // moments of inertia

    double S_wing, AR, b, c, e;            // wing parameters
    double sigma_m, alpha0;                // aerodynamic blending

    // longitudinal parameters
    double C_L_0, C_D_0, C_M_0;
    double C_L_alpha, C_D_alpha, C_M_alpha;
    double C_L_q, C_D_q, C_M_q;

    // lateral parameters
    double C_Y_0, C_ELL_0, C_N_0;
    double C_Y_beta, C_ELL_beta, C_N_beta;
    double C_Y_p, C_ELL_p, C_N_p;
    double C_Y_r, C_ELL_r, C_N_r;

// private attributes
private:

    // dynamics pointer
    DynamicsClass *dynamicsPtr;

    // calculation parameters
    double gamma, gamma1, gamma2, gamma3, gamma4, gamma5, gamma6, gamma7, gamma8;

// public methods
public:

    void read_parameters();
    void write_state();
    void write_dstate();

private:

    void calculate_gamma();

};

#endif
