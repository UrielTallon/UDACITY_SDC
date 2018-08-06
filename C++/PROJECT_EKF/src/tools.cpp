#include <iostream>
#include "tools.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, 
							  const vector<VectorXd> &ground_truth) {

	// Initialize RMSE vector
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// Check for invalidity (different size, empty vector)
	if ((estimations.size() == 0) || (estimations.size() != ground_truth.size())) {
		std::cout << "CalcultateRMSE - Error - Invalid estimation or ground truth" << std::endl;
		return rmse;
	}

	for (unsigned int i(0); i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {

	// Initialiszation of the covariance matrix
	MatrixXd Hj(3, 4);

	// Recover state parameters
	float px(x_state(0));
	float py(x_state(1));
	float vx(x_state(2));
	float vy(x_state(3));

	// Compute intermediate values
	float pxpy0 = pow(px, 2) + pow(py, 2);
	float pxpy1 = sqrt(pxpy0);
	float pxpy2 = pxpy0 * pxpy1;

	// Check division by zero
	if (fabs(pxpy0) < 0.0001) {
		std::cout << "CalculateJacobian - Error - Division by zero" << std::endl;
		return Hj;
	}

	// Compute the jacobian matrix
	Hj << px / pxpy1, py / pxpy1, 0, 0,
		-py / pxpy0, px / pxpy0, 0, 0,
		py*(vx*py - vy*px) / pxpy2, px*(vy*px - vx*py) / pxpy2, px / pxpy1, py / pxpy1;

	return Hj;
}