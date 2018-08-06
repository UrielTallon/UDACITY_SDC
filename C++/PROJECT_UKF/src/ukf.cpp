//#include "stdafx.h" // for Microsoft Visual Studio
#define _USE_MATH_DEFINES // for Microsoft Visual Studio
#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

UKF::UKF() {
	use_laser_ = false;
	
	use_radar_ = false;
	
	previous_timestamp_ = 0;

	NIS_laser_ = 1;

	NIS_radar_ = 1;

	n_x_ = 5;

	n_z_ = 3;

	n_aug_ = n_x_ + 2;

	lambda_ = 3 - n_aug_;

	x_ = VectorXd(n_x_);

	P_ = MatrixXd(n_x_, n_x_);

	std_a_ = 2.5;

	std_yawd_ = 0.6;

	std_laspx_ = 0.15;

	std_laspy_ = 0.15;

	std_radr_ = 0.3;

	std_radphi_ = 0.03;

	std_radrd_ = 0.3;

	weights_ = VectorXd(2 * n_aug_ + 1);

	R_laser_ = MatrixXd(2, 2);
	R_laser_ << std_laspx_ * std_laspx_, 0,
				0, std_laspy_ * std_laspy_;

	R_radar_ = MatrixXd(n_z_, n_z_);
	R_radar_ << std_radr_ * std_radr_, 0, 0,
				0, std_radphi_ * std_radphi_, 0,
				0, 0, std_radrd_ * std_radrd_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_) {

		double x, y;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			x = meas_package.raw_measurements_(0) * cos(meas_package.raw_measurements_(1));
			y = meas_package.raw_measurements_(0) * sin(meas_package.raw_measurements_(1));
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x = meas_package.raw_measurements_[0];
			y = meas_package.raw_measurements_[1];
		}

		if (fabs(x) < 0.001) {
			x = 0.001;
		}
		if (fabs(y) < 0.001) {
			y = 0.001;
		}

		x_ << x, y, 0, 0, 0;
		P_ << MatrixXd::Identity(n_x_, n_x_);

		weights_(0) = lambda_ / (lambda_ + n_aug_);
		for (int i(1); i < 2 * n_aug_ + 1; ++i) {
			weights_(i) = 0.5 / (lambda_ + n_aug_);
		}

		is_initialized_ = true;
		previous_timestamp_ = meas_package.timestamp_;
		return;
	}

	float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = meas_package.timestamp_;

	/***************************************************
	* Generate Augmented Sigma Points
	****************************************************/
	x_aug_ = VectorXd(n_aug_);
	x_aug_.head(n_x_) = x_;
	x_aug_(5) = 0;
	x_aug_(6) = 0;


	MatrixXd Q = MatrixXd(2, 2);
	Q << std_a_ * std_a_, 0,
		0, std_yawd_ * std_yawd_;

	P_aug_ = MatrixXd(n_aug_, n_aug_);
	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(n_x_, n_x_) = P_;
	P_aug_.bottomRightCorner(2, 2) = Q;

	MatrixXd L = P_aug_.llt().matrixL();
	MatrixXd sq_res = sqrt(lambda_ + n_aug_) * L;
	Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	Xsig_aug_.col(0) = x_aug_;
	for (int i(0); i < n_aug_; ++i) {
		Xsig_aug_.col(i + 1) = x_aug_ + sq_res.col(i);
		Xsig_aug_.col(i + n_aug_ + 1) = x_aug_ - sq_res.col(i);
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	Prediction(dt);

	/*****************************************************************************
	*  Update
	****************************************************************************/
	if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		UpdateLidar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
		VectorXd z_pred = VectorXd(n_z_);
		MatrixXd S = MatrixXd(n_z_, n_z_);
		Zsig.fill(0.0);
		z_pred.fill(0.0);
		S.fill(0.0);
		PredictRadarMeasurement(Zsig, z_pred, S);
		UpdateRadar(meas_package, Zsig, z_pred, S);
	}

}

void UKF::Prediction(const double delta_t) {

	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
	x_.fill(0.0);
	P_.fill(0.0);

	for (int i(0); i < 2 * n_aug_ + 1; ++i) {
		double px = Xsig_aug_(0, i);
		double py = Xsig_aug_(1, i);
		double v = Xsig_aug_(2, i);
		double yaw = Xsig_aug_(3, i);
		double yawd = Xsig_aug_(4, i);
		double nu_a = Xsig_aug_(5, i);
		double nu_yawdd = Xsig_aug_(6, i);

		double px_p, py_p;

		if (fabs(yawd) < 0.001) {
			px_p = px + v * delta_t * cos(yaw);
			py_p = py + v * delta_t * sin(yaw);
		}
		else {
			px_p = px + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = py + v / yawd * (-cos(yaw + yawd*delta_t) + cos(yaw));
		}

		VectorXd noise = VectorXd(n_x_);
		VectorXd x_pred = VectorXd(n_x_);

		noise << 0.5 * delta_t * delta_t * cos(yaw) * nu_a,
			0.5 * delta_t * delta_t * sin(yaw) * nu_a,
			delta_t * nu_a,
			0.5 * delta_t * delta_t * nu_yawdd,
			delta_t * nu_yawdd;
		x_pred << px_p, py_p, v, yaw + yawd * delta_t, yawd;

		Xsig_pred_.col(i) = x_pred + noise;
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}

	for (int i(0); i < 2 * n_aug_ + 1; ++i) {
		VectorXd diff = Xsig_pred_.col(i) - x_;
		while (diff(3) > M_PI) diff(3) -= 2. * M_PI;
		while (diff(3) < -M_PI) diff(3) += 2. * M_PI;
		P_ = P_ + weights_(i) * diff * diff.transpose();
	}
}

void UKF::PredictRadarMeasurement(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

	for (int i(0); i < 2 * n_aug_ + 1; ++i) {
		VectorXd sig = Xsig_pred_.col(i);
		if (fabs(sig(0)) < 0.001 && fabs(sig(1)) < 0.001) {
			sig(0) = 0.001;
			sig(1) = 0.001;
		}
		double radr = sqrt(sig(0) * sig(0) + sig(1) * sig(1));
		double radphi = atan2(sig(1), sig(0));
		double radrd = (sig(0) * cos(sig(3)) * sig(2) + sig(1) *sin(sig(3)) * sig(2)) / radr;
		Zsig.col(i) << radr, radphi, radrd;
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	for (int i(0); i < 2 * n_aug_ + 1; ++i) {
		VectorXd diff = Zsig.col(i) - z_pred;
		while (diff(1) > M_PI) diff(1) -= 2. * M_PI;
		while (diff(1) < -M_PI) diff(1) += 2. * M_PI;
		S = S + weights_(i) * diff * diff.transpose();
	}

	S = S + R_radar_;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {

	VectorXd measures = VectorXd(2);
	measures << meas_package.raw_measurements_(0),
		meas_package.raw_measurements_(1);

	MatrixXd H = MatrixXd(2, n_x_);
	H << 1, 0, 0, 0, 0,
		0, 1, 0, 0, 0;

	VectorXd x = H * x_;
	VectorXd y = measures - x;
	MatrixXd PHt = P_ * H.transpose();
	MatrixXd S = H * PHt + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd K = PHt * Si;

	x_ = x_ + (K * y);

	long long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H) * P_;

	NIS_laser_ = (x - measures).transpose() * Si * (x - measures);
}

void UKF::UpdateRadar(MeasurementPackage meas_package, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

	VectorXd measures = VectorXd(n_z_);
	measures << meas_package.raw_measurements_(0),
		meas_package.raw_measurements_(1),
		meas_package.raw_measurements_(2);

	MatrixXd Tc = MatrixXd(n_x_, n_z_);
	Tc.fill(0.0);
	for (int i(0); i < 2 * n_aug_ + 1; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	MatrixXd K = Tc * S.inverse();

	VectorXd y = measures - z_pred;
	while (y(1) > M_PI) y(1) -= 2. * M_PI;
	while (y(1) < -M_PI) y(1) += 2. * M_PI;

	x_ = x_ + (K * y);
	P_ = P_ - (K * S * K.transpose());

	NIS_radar_ = (z_pred - measures).transpose() * S.inverse() * (z_pred - measures);
}

void UKF::SetUseLaser() {
	use_laser_ = true;
	std::cout << "Laser enabled!" << std::endl;
}

void UKF::SetUseRadar() {
	use_radar_ = true;
	std::cout << "Radar enabled!" << std::endl;
}