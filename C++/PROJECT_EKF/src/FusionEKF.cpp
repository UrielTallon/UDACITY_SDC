//#include "stdafx.h" // for Microsoft Visual Studio
#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

FusionEKF::FusionEKF() {
	is_initialized_ = false;
	previous_timestamp_ = 0;
	
	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	// measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

	// measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

	// measurement matrix
	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	// state vector
	ekf_.x_ = VectorXd(4);

	// state transition matrix
	ekf_.F_ = MatrixXd(4, 4);

	// state covariance matrix
	ekf_.P_ = MatrixXd(4, 4);

	// process covariance matrix
	ekf_.Q_ = MatrixXd(4, 4);

	// set acceleration and noise components
	noise_ax = 9.0;
	noise_ay = 9.0;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurment_pack) {

	/***************************************************
	 * Initialization                                  *
	 ***************************************************/
	if (!is_initialized_) {

		// first measurement
		float x;
		float y;

		if (measurment_pack.sensor_type_ == MeasurementPackage::RADAR) {
			float phi = measurment_pack.raw_measurements_[1];
			x = measurment_pack.raw_measurements_[0] * cos(phi); // project rho on X axis
			y = measurment_pack.raw_measurements_[0] * sin(phi); // project rho on Y axis
		}
		else if (measurment_pack.sensor_type_ == MeasurementPackage::LASER) {
			x = measurment_pack.raw_measurements_[0];
			y = measurment_pack.raw_measurements_[1];
		}

		// no suitable measurement for initialization
		if ((fabs(x) < 0.0001) || (fabs(y) < 0.0001)) {
			return;
		}
		
		// initialize both the state matrix and the state covariance matrix
		ekf_.x_ << x, y, 0, 0;
		ekf_.P_ << MatrixXd::Identity(4, 4);

		// done with initialization; no need for prediction and/or update
		is_initialized_ = true;
		previous_timestamp_ = measurment_pack.timestamp_;
		return;
	}

	// compute delta t (elapsed between previous and current measurement in s)
	float dt = (measurment_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurment_pack.timestamp_;
	
	/***************************************************
	 * Prediction                                      *
	 ***************************************************/
	ekf_.SetParametersForPrediction(dt, noise_ax, noise_ay);

	ekf_.Predict();

	/***************************************************
	 * Update                                          *
	 ***************************************************/
	if (measurment_pack.sensor_type_ == MeasurementPackage::LASER) {
		MatrixXd H_in = H_laser_;
		MatrixXd R_in = R_laser_;
		ekf_.SetParametersForUpdate(H_in, R_in);
		ekf_.Update(measurment_pack.raw_measurements_);
	}
	else if (measurment_pack.sensor_type_ == MeasurementPackage::RADAR) {
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		MatrixXd R_in = R_radar_;
		ekf_.SetParametersForUpdate(Hj_, R_in);
		ekf_.UpdateEKF(measurment_pack.raw_measurements_);
	}

}

