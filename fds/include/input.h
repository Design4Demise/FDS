#ifndef INPUT_H
#define INPUT_H

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <vector>
#include <array>
#include <Eigen/Dense>

const double rho = 1.225;
const double g0  = 9.81;

const double time_step = 1e-3;

enum eQuaternion {e_e0, e_e1, e_e2, e_e3};
enum eEuler {e_phi, e_theta, e_psi};
enum eState {es_px, es_py, es_pz, es_u, es_v, es_w, es_e0, es_e1, es_e2, es_e3, es_p, es_q, es_r};

#define SQ(x) ((x) * (x))

#endif
