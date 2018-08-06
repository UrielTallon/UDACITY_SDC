#ifndef UKF_H_
#define UKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class UKF {
public:
	bool is_initialized_;

	bool use_laser_;

	bool use_radar_;

	VectorXd weights_;

	VectorXd x_;

	VectorXd x_aug_;

	MatrixXd P_;

	MatrixXd P_aug_;

	MatrixXd Xsig_pred_;

	MatrixXd Xsig_aug_;

	MatrixXd R_laser_;

	MatrixXd R_radar_;

	long long time_us_;

	long long previous_timestamp_;

	double std_a_;

	double std_yawd_;

	double std_laspx_;

	double std_laspy_;

	double std_radr_;

	double std_radphi_;

	double std_radrd_;

	int n_x_;

	int n_aug_;

	int n_z_;

	double lambda_;

	double NIS_radar_;

	double NIS_laser_;

	UKF();

	virtual ~UKF();

	void ProcessMeasurement(MeasurementPackage meas_package);

	void Prediction(double delta_t);

	void UpdateLidar(MeasurementPackage meas_package);

	void UpdateRadar(MeasurementPackage meas_package, MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);

	void PredictRadarMeasurement(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S);

	void SetUseLaser();

	void SetUseRadar();
};

#endif // !UKF_H_
