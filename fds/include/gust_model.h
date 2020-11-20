#ifndef GUST_H
#define GUST_H

#include "input.h"
#include "wind.h"
#include "transfer_function.h"
#include "dynamics.h"

class WindClass;
class DynamicsClass;
class TransferFunction;

class BaseGustModel {

	friend class WindClass;
	friend class DynamicsClass;

public:
	virtual void update_gust() = 0;

};

class DrydenGustModel : public BaseGustModel {

// con/de-structors
public:

	DrydenGustModel(WindClass &wind);
	~DrydenGustModel();

// public attributes
public:

	Eigen::Vector3d gust;

	// for now lets skip over V_a
	double v_a = 25.0;

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

	void setup_tf();

};

class VonKarmanGustModel : public BaseGustModel {

// con/de-structors
public:

	VonKarmanGustModel();
	~VonKarmanGustModel();

// public attributes
public:

// private attributes
private:

// public methods
	void update_gust();

};

#endif
