#include "../include/uav.h"
#include "../include/dynamics.h"

int main() {

    std::cout << "Flight Dynamics Simulation" << std::endl;

    UAVClass uav;
    DynamicsClass dyn(uav);

    for (dyn.curr_time_step = 0; dyn.curr_time_step < nSteps; dyn.curr_time_step++) {

    	dyn.update();

    	if (dyn.curr_time_step % logging_interval == 0) {
    		uav.write_state();
    		uav.write_dstate();
    	}
    
    }

    std::cout << "Simulation Complete." << std::endl;

    return 0;
}
