#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

///<summary>
///<para>The class GroundTruthPackage defines a data structure used to</para>
///<para>store the actual values for (x, y) position and (vx, vy) velocity.</para>
///</summary>
///<remark>This is used for RMSE calculation.</remarks>
class GroundTruthPackage {
public:
	///<summary>The timestamp for current state (in microseconds).</summary>
	long long timestamp_;

	///<summary>The type of the sensor for current state.</summary>
	enum SensorType {
		LASER,
		RADAR
	} sensor_type_;

	///<summary>The current ground truth values.</summary>
	Eigen::VectorXd gt_values_;
};

#endif // !GROUND_TRUTH_PACKAGE_H_

