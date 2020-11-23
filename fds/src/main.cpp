#include "../include/uav.h"
#include "../include/dynamics.h"
#include "../include/wind.h"
#include "../include/propulsion.h"

int main() {

    std::cout << "Flight Dynamics Simulation" << std::endl;

    UAVClass uav;
    PropulsionClass propulsion(uav);

    WindClass wind(true);
    DynamicsClass dyn(uav, wind);

    for (dyn.curr_time_step = 0; dyn.curr_time_step < nSteps; dyn.curr_time_step++) {

        dyn.update();
        wind.update_gust();

        if (dyn.curr_time_step % logging_interval == 0) {
            uav.write_state();
            uav.write_dstate();
        }
    
    }
        
    std::cout << "Simulation Complete." << std::endl;

    return 0;

}
