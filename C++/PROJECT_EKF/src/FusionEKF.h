#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

///<summary>
///<para>The FusionEKF class is used to process the different measurements. The class</para>
///<para>uses an instance of the KalmanFilter class to make calls to the Predict and</para>
///<para>Update methods.</para>
///</summary>
class FusionEKF {
public:
	///<summary>
	///FusionEKF class constructor.
	///</summary>
	FusionEKF();

	///<summary>
	///FusionEKF class destructor.
	///</summary>
	virtual ~FusionEKF();

	///<summary>
	///<para>Method of the FusionEKF class to process the measurements. The first measurement is
	///used to initialize the</para><para>state vector, provided the collected measurements are
	///valid (i.e. not null). During subsequent measurements</para><para>the elapsed time is
	///calculated and both predictions and updates are made.</para>
	///</summary>
	///<param name="measurement_pack">
	///A MeasurementPackage class instance containing raw measurements for a gicin timestamp.
	///</param>
	///<returns>Void.</returns>
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	///<summary>
	///<para>An instance of the KalmanFilter class, used to access the different vectors</para>
	///<para>and methods for predictions and update.</para>
	///</summary>
	KalmanFilter ekf_;

private:
	///<summary>Check wether the state vector is initialized (first measurement).</summary>
	bool is_initialized_;

	///<summary>Previous timestamp.</summary>
	///<remarks>The type <c>long long</c> is required to avoid errors.</remarks>
	long long previous_timestamp_;

	///<summary>Noise component along X axis.</summary>
	float noise_ax;
	///<summary>Noise component along Y axis.</summary>
	float noise_ay;

	///<summary>An instance of the Tools class used to access the methods for RMSE and Jacobian computation.</summary>
	Tools tools;
	
	///<summary>Measurement covariance matrix for LASER measurement.</summary>
	///<remarks>Dimension is (2, 2).</remarks>
	Eigen::MatrixXd R_laser_;
	
	///<summary>Measurement covariance matrix for RADAR measurement.</summary>
	///<remarks>Dimension is (3, 3).</remarks>
	Eigen::MatrixXd R_radar_;

	///<summary>Measurement matrix for LASER measurements.</summary>
	Eigen::MatrixXd H_laser_;

	///<summary>Jacobian matrix for RADAR measurements.</summary>
	Eigen::MatrixXd Hj_;
};

#endif // !FUSION_EKF_H_

