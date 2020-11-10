#ifndef UTILS_H
#define UTILS_H

#include "input.h"

namespace utils {

std::array<double, 3> Quaternion2Euler(std::array<double, 4> q);
Eigen::Matrix3d Quaternion2Rotation(std::array<double, 4> q);


std::array<double, 4> Euler2Quaternion(double phi, double theta, double psi);
Eigen::Matrix3d Euler2Rotation(double phi, double theta, double psi);

void create_directories();

}

#endif
