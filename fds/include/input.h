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

enum eQuaternion {e_e0, e_e1, e_e2, e_e3};
enum eEuler {e_phi, e_theta, e_psi};

#define SQ(x) ((x) * (x))

#endif
