#ifndef INPUT_H
#define INPUT_H

#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Core>

// physical parameters
const double rho = 1.225;
const double g0  = 9.81;

// simulation parameters
const double t_simulation = 30.0;
const double time_step = 1e-3;

const int nSteps = static_cast<int>(t_simulation / time_step);

// logging parameters
const int logging_interval = nSteps / 1000;
const int PRECISION = 10;

// enum definitions
enum eDirection {ex, ey, ez};
enum eForce {e_fx, e_fy, e_fz};

enum eEuler {e_phi, e_theta, e_psi};
enum eQuaternion {e_e0, e_e1, e_e2, e_e3};

enum eForceMoment {efm_fx, efm_fy, efm_fz, efm_L, efm_M, efm_N};
enum eState {es_px, es_py, es_pz, es_u, es_v, es_w, es_e0, es_e1, es_e2, es_e3, es_p, es_q, es_r};

enum eWind {ew_u, ew_v, ew_w, ew_gust_u, ew_gust_v, ew_gust_w};

// macro definitions
#define SQ(x) ((x) * (x))

#endif
