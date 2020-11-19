#ifndef WIND_H
#define WIND_H


#include "input.h"

class WindClass{

	//constructors deconstructors
public:

	WindClass();
	~WindClass() {};

	WindClass(bool read_params);

	//public attributes
public:

    // std::array<double, 6> windState;
    Eigen::Matrix<double, 6, 1> windState;

	//public methods
public:
	void read_parameters();

};



#endif