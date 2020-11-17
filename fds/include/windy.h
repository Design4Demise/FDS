#ifndef WINDY_H
#define WINDY_H


#include "input.h"

class WindyClass{

	//constructors deconstructors
public:

	WindyClass();
	~WindyClass() {};

	WindyClass(bool read_params);

	//public attributes
public:

	double u, v, w
	double gust_u, gust_v, gust_w

    std::array<double, 6> windState

	//private attributes
private:




	//public methods
public:
	void read_parameters()

};



#endif