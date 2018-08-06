// CarND_ExtendedKalmanFilter.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h" // for Microsoft Visual Studio
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
	std::string usage_instructions = "Usage instructions: ";
	usage_instructions += argv[0];
	usage_instructions += " path_to_input_file path_to_output_file";

	bool has_valid_args = false;

	// make sure the user has provided input and output files
	if (argc == 1) {
		std::cerr << usage_instructions << std::endl;
	}
	else if (argc == 2) {
		std::cerr << "Please include an output file.\n" << usage_instructions << std::endl;
	}
	else if (argc == 3) {
		has_valid_args = true;
	}
	else if (argc > 3) {
		std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
	}

	if (!has_valid_args) {
		std::exit(EXIT_FAILURE);
	}
}

void check_files(std::ifstream &in_file, std::string &in_name,
				 std::ofstream &out_file, std::string &out_name) {
	
	if (!in_file.is_open()) {
		std::cerr << "Cannot open input file: " << in_name << std::endl;
		std::exit(EXIT_FAILURE);
	}

	if (!out_file.is_open()) {
		std::cerr << "Cannot open output file: " << out_name << std::endl;
		std::exit(EXIT_FAILURE);
	}
}

int main(int argc, char* argv[])
{

	check_arguments(argc, argv);

	std::string in_file_name_ = argv[1];
	std::ifstream in_file_(in_file_name_.c_str(), std::ifstream::in);

	std::string out_file_name_ = argv[2];
	std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);

	check_files(in_file_, in_file_name_, out_file_, out_file_name_);

	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;

	std::string line;

	// Prepare the measurment package (each line represent a measurement at a timestamp)
	while (std::getline(in_file_, line)) {

		std::string sensor_type;
		MeasurementPackage meas_package;
		GroundTruthPackage gt_package;
		std::istringstream iss(line);
		long long timestamp;

		// read the first element from the current line (sensor type)
		iss >> sensor_type;
		if (sensor_type.compare("L") == 0) { // LASER MEASUREMENT
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x, y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}
		else if (sensor_type.compare("R") == 0) { // RADAR MEASUREMENT
			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			float ro;
			float phi;
			float ro_dot;
			iss >> ro;
			iss >> phi;
			iss >> ro_dot;
			meas_package.raw_measurements_ << ro, phi, ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}

		// Read ground truth data for later comparison
		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		gt_package.gt_values_ = VectorXd(4);
		gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
		gt_pack_list.push_back(gt_package);
	}

	// Create a FusionEKF instance
	FusionEKF fusionEKF;

	// For RMSE computaions
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	// Call the EKF-based fusion
	std::size_t N = measurement_pack_list.size();
	for (std::size_t k(0); k < N; ++k) {
		
		// NOTA: filtering begins on the 2nd frame (speed is unknown in the first frame)
		fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

		// Output estimations
		out_file_ << fusionEKF.ekf_.x_(0) << "\t";
		out_file_ << fusionEKF.ekf_.x_(1) << "\t";
		out_file_ << fusionEKF.ekf_.x_(2) << "\t";
		out_file_ << fusionEKF.ekf_.x_(3) << "\t";

		// Output the measurements
		if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
			out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
			out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
		}
		else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
			float ro = measurement_pack_list[k].raw_measurements_(0);
			float phi = measurement_pack_list[k].raw_measurements_(1);
			out_file_ << ro * cos(phi) << "\t"; // p1_meas
			out_file_ << ro * sin(phi) << "\t"; // ps_meas
		}

		// Output the ground truth package
		out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

		estimations.push_back(fusionEKF.ekf_.x_);
		ground_truth.push_back(gt_pack_list[k].gt_values_);
	}

	// Compute the accuracy (RMSE)
	Tools tools;
	std::cout << "RMSE" << std::endl << tools.CalculateRMSE(estimations, ground_truth) << std::endl;

	// Close files
	if (in_file_.is_open()) {
		in_file_.close();
	}

	if (out_file_.is_open()) {
		out_file_.close();
	}

    return 0;
}

