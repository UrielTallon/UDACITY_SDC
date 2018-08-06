// CarND_UnscentedKalmanFilter.cpp : Defines the entry point for the console application.
//

#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "ukf.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
	std::string usage_instructions = "Usage instructions: ";
	usage_instructions += argv[0];
	usage_instructions += " path_to_input_file path_to_output_file";
	usage_instructions += " -l/-r/-b";

	bool has_valid_args = false;

	// make sure the user has provided input and output files
	if (argc == 1) {
		std::cerr << usage_instructions << std::endl;
	}
	else if (argc == 2) {
		std::cerr << "Please include an output file.\n" << usage_instructions << std::endl;
	}
	else if (argc == 3) {
		std::cerr << "Please select a sensor.\n" << usage_instructions << std::endl;
	}
	else if (argc == 4) {
		has_valid_args = true;
	}
	else if (argc > 4) {
		std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
	}

	if (!has_valid_args) {
		exit(EXIT_FAILURE);
	}
}

void check_files(std::ifstream& in_file, std::string& in_name,
				 std::ofstream& out_file, std::string& out_name) {
	if (!in_file.is_open()) {
		std::cerr << "Cannot open input file: " << in_name << std::endl;
		exit(EXIT_FAILURE);
	}

	if (!out_file.is_open()) {
		std::cerr << "Cannot open output file: " << out_name << std::endl;
		exit(EXIT_FAILURE);
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

	UKF ukf;

	std::string selected_sensor_ = argv[3];
	if (selected_sensor_.compare("-b") == 0) {
		ukf.SetUseLaser();
		ukf.SetUseRadar();
	}
	else if (selected_sensor_.compare("-l") == 0) {
		ukf.SetUseLaser();
	}
	else if (selected_sensor_.compare("-r") == 0) {
		ukf.SetUseRadar();
	}
	else {
		std::cerr << "Invalid sensor selection: " << selected_sensor_ << std::endl 
			<< "Choose between '-l', '-r' or '-b'" << std::endl;
		exit(EXIT_FAILURE);
	}

	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;

	std::string line;

	while (getline(in_file_, line)) {
		std::string sensor_type;
		MeasurementPackage meas_package;
		GroundTruthPackage gt_package;
		std::istringstream iss(line);
		long long timestamp;
		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;

		iss >> sensor_type;

		if (ukf.use_laser_ && (sensor_type.compare("L") == 0)) {
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float px;
			float py;
			iss >> px;
			iss >> py;
			meas_package.raw_measurements_ << px, py;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
			iss >> x_gt;
			iss >> y_gt;
			iss >> vx_gt;
			iss >> vy_gt;
			gt_package.gt_values_ = VectorXd(4);
			gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
			gt_pack_list.push_back(gt_package);
		}
		else if (ukf.use_radar_ && (sensor_type.compare("R") == 0)) {
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
			iss >> x_gt;
			iss >> y_gt;
			iss >> vx_gt;
			iss >> vy_gt;
			gt_package.gt_values_ = VectorXd(4);
			gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
			gt_pack_list.push_back(gt_package);
		}
	}

	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	size_t number_of_measurements = measurement_pack_list.size();

	out_file_ << "time_stamp" << "\t";
	out_file_ << "px_state" << "\t";
	out_file_ << "py_state" << "\t";
	out_file_ << "v_state" << "\t";
	out_file_ << "yaw_angle_state" << "\t";
	out_file_ << "yaw_rate_state" << "\t";
	out_file_ << "sensor_type" << "\t";
	out_file_ << "NIS" << "\t";
	out_file_ << "px_measured" << "\t";
	out_file_ << "py_measured" << "\t";
	out_file_ << "px_ground_truth" << "\t";
	out_file_ << "py_ground_truth" << "\t";
	out_file_ << "vx_ground_truth" << "\t";
	out_file_ << "vy_ground_truth" << "\n";

	for (size_t k = 0; k < number_of_measurements; ++k) {
		ukf.ProcessMeasurement(measurement_pack_list[k]);

		out_file_ << measurement_pack_list[k].timestamp_ << "\t";

		out_file_ << ukf.x_(0) << "\t";
		out_file_ << ukf.x_(1) << "\t";
		out_file_ << ukf.x_(2) << "\t";
		out_file_ << ukf.x_(3) << "\t";
		out_file_ << ukf.x_(4) << "\t";

		if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
			out_file_ << "lidar" << "\t";
			out_file_ << ukf.NIS_laser_ << "\t";
			out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
			out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
		}
		else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
			out_file_ << "radar" << "\t";
			out_file_ << ukf.NIS_radar_ << "\t";
			float ro = measurement_pack_list[k].raw_measurements_(0);
			float phi = measurement_pack_list[k].raw_measurements_(1);
			out_file_ << ro * cos(phi) << "\t";
			out_file_ << ro * sin(phi) << "\t";
		}

		out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

		VectorXd ukf_x_cartesian_ = VectorXd(4);

		float x_estimate_ = ukf.x_(0);
		float y_estimate_ = ukf.x_(1);
		float vx_estimate_ = ukf.x_(2) * cos(ukf.x_(3));
		float vy_estimate_ = ukf.x_(2) * sin(ukf.x_(3));

		ukf_x_cartesian_ << x_estimate_, y_estimate_, vx_estimate_, vy_estimate_;

		estimations.push_back(ukf_x_cartesian_);
		ground_truth.push_back(gt_pack_list[k].gt_values_);

	}

	Tools tools;
	std::cout << "RMSE" << std::endl << tools.CalculateRMSE(estimations, ground_truth) << std::endl;

	if (out_file_.is_open()) {
		out_file_.close();
	}

	if (in_file_.is_open()) {
		in_file_.close();
	}

	std::cout << "Done!" << std::endl;
    return 0;
}

