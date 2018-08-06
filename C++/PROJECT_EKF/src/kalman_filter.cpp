//#include "stdafx.h" // for Microsoft Visual Studio
#include "kalman_filter.h"
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::SetParametersForPrediction(const float &dt, const float &nx, const float &ny) {

	// set the state transition matrix
	F_ = MatrixXd::Identity(4, 4);
	F_(0, 2) = dt;
	F_(1, 3) = dt;

	// set the process covariance matrix
	Q_ << (pow(dt, 4) / 4)*nx, 0, (pow(dt, 3) / 2)*nx, 0,
		  0, (pow(dt, 4) / 4)*ny, 0, (pow(dt, 3) / 2)*ny,
		  (pow(dt, 3) / 2)*nx, 0, pow(dt, 2)*nx, 0,
		  0, (pow(dt, 3) / 2)*ny, 0, pow(dt, 2)*ny;

}

void KalmanFilter::SetParametersForUpdate(const MatrixXd &H_in, const MatrixXd &R_in) {

	H_ = H_in;
	R_ = R_in;

}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;
	MatrixXd S = H_ * PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	// new estimation
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	// coefficient for the non-linear radar measurement function
	float map0 = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
	float map1 = atan2(x_(1), x_(0));
	float map2 = (x_(0)*x_(2) + x_(1)*x_(3)) / map0;
	// radar measurment function
	MatrixXd z_pred = MatrixXd(3, 1);
	z_pred << map0, map1, map2;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd PHt = P_ * Ht;
	MatrixXd S = H_ * PHt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	// new estimations
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}