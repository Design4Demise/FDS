#include "../include/uav.h"
#include "../include/dynamics.h"

int main() {

    std::cout << "Flight Dynamics Simulation" << std::endl;

    UAVClass uav;
    DynamicsClass dyn(uav);

    std::cout << "DONE" << std::endl;

    return 0;
}
