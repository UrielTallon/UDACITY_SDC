#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

///<summary>
///<para>The class MeasurementPackage defines a data structure used to</para>
///<para>store the values collected by the sensors.</para>
///</summary>
class MeasurementPackage {
public:
	///<summary>The timestamp for current measurement (in microseconds).</summary>
	long long timestamp_;

	///<summary>The type of the sensor for current measurement.</summary>
	enum SensorType {
		LASER,
		RADAR
	} sensor_type_;

	///<summary>The raw current measurements.</summary>
	Eigen::VectorXd raw_measurements_;
};

#endif // !MEASUREMENT_PACKAGE_H_

