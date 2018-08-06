#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

///<summary>
///<para>The KalmanFilter class is used to store the diifferent vectors and matrices</para>
///<para>pertaining to the states and measurements. Class methods are used to make</para>
///<para>predictions and updates accordingly.</para>
///</summary>
class KalmanFilter {
public:

	///<summary>State vector.</summary>
	///<remarks>This vector is updated over time with each update and predcition step.</remarks>
	Eigen::VectorXd x_;

	///<summary>State covariance matrix.</summary>
	///<remarks>This matrix is updated over time with each update and predcition step.</remarks>
	Eigen::MatrixXd P_;

	///<summary>State transition matrix.</summary>
	Eigen::MatrixXd F_;

	///<summary>Process covariance matrix.</summary>
	Eigen::MatrixXd Q_;

	///<summary>Measurement matrix.</summary>
	Eigen::MatrixXd H_;

	///<summary>Measurement covariance matrix.</summary>
	///<remarks>The dimension of this matrix is different for laser (2x2) and radar (3x3).</remarks>
	Eigen::MatrixXd R_;

	///<summary>
	///KalmanFilter class constructor.
	///</summary>
	KalmanFilter();

	///<summary>
	///KalmanFilter class destructor.
	///</summary>
	virtual ~KalmanFilter();

	
	///<summary>
	///<para>Method of the KalmanFilter class to initialize the parameters of the filter
	///before prediction.</para><para>Integrates the elapsed time into the state transition matrix
	///F and calculates the process covariance matrix Q.</para>
	///</summary>
	///<param name="dt">Elapsed time between two measurements.</param>
	///<param name="nx">Noise along X axis.</param>
	///<param name="ny">Noise along Y axis.</param>
	///<returns>Void.</returns>
	void SetParametersForPrediction(const float &dt, const float &nx, const float &ny);

	///<summary>
	///<para>Method of the KalmanFilter class to initialize the parameters of the filter
	///before update.</para><para>Sets both the measurement matrix H and the measurement
	///covariance matrix R.</para>
	///</summary>
	///<param name="H_in">Value of the measurment matrix.</param>
	///<param name="R_in">Value of the measurement covariance matrix.</param>
	///<returns>Void.</returns>
	void SetParametersForUpdate(const Eigen::MatrixXd &H_in, const Eigen::MatrixXd &R_in);

	///<summary>Method of the KalmanFilter class to predict the current state.</summary>
	///<returns>Void.</returns>
	void Predict();

	///<summary>Updates the state by using the standard Kalman filter equations.</summary>
	///<remarks>Use this for LASER measurements.</remarks>
	///<param name="z">Measurement at k+1.</param>
	///<returns>Void.</returns>
	void Update(const Eigen::VectorXd &z);

	///<summary>Updates the state by using the extended Kalman filter equations.</summary>
	///<remarks>Use this for RADAR measurements.</remarks>
	///<param name="z">Measurement at k+1.</param>
	///<returns>Void.</returns>
	void UpdateEKF(const Eigen::VectorXd &z);

};

#endif // !KALMAN_FILTER_H_

