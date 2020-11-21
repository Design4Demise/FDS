#ifndef WIND_H
#define WIND_H

#include "input.h"
#include "gust_model.h"
#include "dynamics.h"


// forward declarations
class BaseGustModel;
class DrydenGustModel;
class DynamicsClass;


class WindClass{

    // friend class
    friend class DynamicsClass;
    friend class DrydenGustModel;

// ctor/dtor
public:

    WindClass();
    ~WindClass() {};

    WindClass(bool read_params);

// public attributes
public:

    // Consider :: When to use Eigen::Matrix vs. std::array
    Eigen::Matrix<double, 6, 1> windState;
    BaseGustModel *gustPtr;

// private attributes
private:

    DynamicsClass *dynamicsPtr;

// public methods
public:

    void read_parameters();     // read parameters from file
    void update_gust();         // update gust model

// private methods
private:

    void set_gust_model();

};

#endif
