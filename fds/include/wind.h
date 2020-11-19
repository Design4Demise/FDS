#ifndef WIND_H
#define WIND_H


#include "input.h"

class WindClass{

//con/de-structors
public:

	WindClass();
	~WindClass() {};

	WindClass(bool read_params);

//public attributes
public:

	// Consider :: When to use Eigen::Matrix vs. std::array
    Eigen::Matrix<double, 6, 1> windState;

//public methods
public:

	void read_parameters();

};

#endif
