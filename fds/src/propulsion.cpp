#include "../include/propulsion.h"


PropulsionClass::PropulsionClass(UAVClass &uav) {

	uavPtr = &uav;
	uavPtr->propulsionPtr = this;

	thrust = 0.0;
	torque = 0.0;

}

void PropulsionClass::update_motor() {
	
	double v_air = uavPtr->dynamicsPtr->v_air;
	double v_in = uavPtr->Vmax * uavPtr->controlState[ec_delta_t];

	double a = uavPtr->C_Q0 * rho * std::pow(uavPtr->D_prop, 5.0) / SQ(2.0 * M_PI);
	double b = (uavPtr->C_Q1 * rho * std::pow(uavPtr->D_prop, 4.0) / (2.0 * M_PI)) * v_air + SQ(uavPtr->KQ) / uavPtr->R_motor;
	double c = uavPtr->C_Q2 * rho * std::pow(uavPtr->D_prop, 3.0) * SQ(v_air) - v_in * (uavPtr->KQ / uavPtr->R_motor) + uavPtr->KQ * uavPtr->i0;

	double omega = (-b + std::sqrt(SQ(b) - 4.0 * a * c)) / (2.0 * a);
	double J = 2.0 * M_PI * v_air / (omega * uavPtr->D_prop);

	double C_T = uavPtr->C_T2 * SQ(J) + uavPtr->C_T1 * J + uavPtr->C_T0;
	double C_Q = uavPtr->C_Q2 * SQ(J) + uavPtr->C_Q1 * J + uavPtr->C_Q0;

	double n = omega / (2 * M_PI);

	thrust = rho * SQ(n) * std::pow(uavPtr->D_prop, 4.0) * C_T;
	torque = rho * SQ(n) * std::pow(uavPtr->D_prop, 5.0) * C_Q;

}
