#ifndef PROPULSION_H
#define PROPULSION_H

#include "input.h"
#include "uav.h"

// forward declaration
class UAVClass;


class PropulsionClass {

// ctor/dtor
public:

	~PropulsionClass() {};

	PropulsionClass(UAVClass &uav);

// public attributes
public:

	double thrust;
	double torque;

// private attributes
private:

	UAVClass *uavPtr;

// public methods
public:

	void update_motor();

};

#endif
