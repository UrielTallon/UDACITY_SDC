//#include "stdafx.h" // for Microsoft Visual Studio
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