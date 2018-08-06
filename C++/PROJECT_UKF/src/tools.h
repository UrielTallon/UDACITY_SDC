#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

///<summary>
///<para>The Tools class defines some useful helper methods for RMSE computation.</para>
///</summary>
class Tools {
public:
	///<summary>
	///Tools class constructor.
	///</summary>
	Tools();

	///<summary>
	///Tools class destructor.
	///</summary>
	virtual ~Tools();

	///<summary>
	///<para>Method of the Tools class to compute the RMSE for each component of the state vector
	///(px, py, vx, vy).</para><para>The method checks the validity of the input parameters (different
	///sizes or empty estimations).</para>
	///</summary>
	///<param name="estimations">A collection of estimations predicted by the Kalman filter.</param>
	///<param name="ground_truth">The actual values for each component of the state vector.</param>
	///<returns>
	///<para>A vector of size 4 containing the RMSE for each component (px, py, vx, vy).</para>
	///<para>If the inputs are invalid, the returned vector is (0, 0, 0, 0).</para>
	///</returns>
	Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
								  const std::vector<Eigen::VectorXd> &ground_truth);
};

#endif // !TOOLS_H_

