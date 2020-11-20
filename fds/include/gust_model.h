#ifndef GUST_H
#define GUST_H

#include "input.h"
#include "wind.h"
#include "transfer_function.h"


// forward declarations
class WindClass;


// abstract base class
class BaseGustModel {

public:
	virtual void update_gust() = 0;

};


class DrydenGustModel : public BaseGustModel {

// ctor/dtor
public:

	DrydenGustModel(WindClass &wind);
	~DrydenGustModel();

// public attributes
public:

	Eigen::Vector3d gust;

	// scale lengths
	const double lu = 200;
	const double lv = 200;
	const double lw = 50;

	// gust intensity
	const double sigma_u = 1.06;
	const double sigma_v = 1.06;
	const double sigma_w = 0.7;

// private attributes
private:

	WindClass *windPtr;
	bool updated_vel;

	TransferFunction h_u;
	TransferFunction h_v;
	TransferFunction h_w;

	std::random_device rd{};
    std::mt19937 generator{rd()};
    std::normal_distribution<double> dist{0.0, 1.0};

// public methods
public:

	void update_gust();

// private methods
private:

	void update_tf();

};


// unimplemented thus far
class VonKarmanGustModel : public BaseGustModel {

// ctor/dtor
public:

	VonKarmanGustModel();
	~VonKarmanGustModel();

// public attributes
public:

// private attributes
private:

// public methods
public:

	void update_gust();

};

#endif
