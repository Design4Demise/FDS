#ifndef TF_H
#define TF_H

#include "../include/input.h"


class TransferFunction {

// ctor/dtor
public:

	TransferFunction();
	~TransferFunction() {};

	TransferFunction(Eigen::RowVectorXd numerator, Eigen::RowVectorXd denominator, double tstep);

// public attributes
public:

	Eigen::RowVectorXd num;
	Eigen::RowVectorXd den;
	double ts;
	
// private attributes
private:

	int n;

	Eigen::VectorXd state;

	Eigen::MatrixXd A;
	Eigen::VectorXd B;
	Eigen::RowVectorXd C;

// public methods
public:

	double update(double u);

};

#endif
