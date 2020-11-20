#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "input.h"
#include "uav.h"
#include "wind.h"

// forward declaration
class UAVClass;
class WindClass;

class DynamicsClass {

    // friend class
    friend class WindClass;

// con/de-structors
public:

    DynamicsClass();
    ~DynamicsClass() {};

    DynamicsClass(UAVClass &uav);

    DynamicsClass(UAVClass &uav, WindClass &wind);

// public attributes
public:

    UAVClass *uavPtr;
    WindClass *windPtr;

    int curr_time_step;           // current time step
    double v_air;                 // freestream air velocity

    double alpha;                 // angle of attack
    double beta;                  // sideslip angle

// private attributes
private:
    
    std::array<double, 6> forces_moments;

// public methods
public:

    void update();                // update state of aircraft
    void update_velocity();            // update wind state

// private methods
private:

    Eigen::Matrix<double, 13, 1> calc_derivatives(Eigen::Matrix<double, 13, 1> in_state);
    void calc_forces_moments();

    bool below_groundplane();

};

#endif
