#include "../include/transfer_function.h"

TransferFunction::TransferFunction() {

	ts = time_step;
	num = Eigen::RowVectorXd::Zero(3);
	den = Eigen::RowVectorXd::Zero(3);

}

TransferFunction::TransferFunction(Eigen::RowVectorXd numerator, Eigen::RowVectorXd denominator, double tstep) {

	// check numerator indices >= denominator indices
	if (denominator.cols() < numerator.cols()) {
		std::cout << "Power of Denominator must equal or exceed that of the Numerator.";
		std::exit(EXIT_FAILURE);
	}

	// set attributes
	ts = tstep;
	den = denominator;

	// add leading zeros to numerator
	num.resize(den.cols());
	num.rightCols(numerator.cols()) = numerator;

	// matrix size
	n = num.cols() - 1;

	// initialise state
	state = Eigen::VectorXd::Zero(n);
	
	// initialise control matrices
	A = Eigen::MatrixXd::Zero(n, n);
	B = Eigen::VectorXd::Zero(n);
	C = Eigen::RowVectorXd::Zero(n);

	// set A
	for (int i = 0; i < n - 1; i++)
		A(i, i + 1) = 1.0;

	for (int i = 0; i < n; i++)
		A(n - 1, i) = - den(n - i);

	// set B
	B(n - 1) = 1.0;

	// set C
	for (int i = 0; i < n; i++)
		C(i) = num(n - i) - den(n - i) * num(0);

}

double TransferFunction::update(double u) {

	state = state + ts * (A * state + B * u);
	return C * state + num(0) * u;

}
