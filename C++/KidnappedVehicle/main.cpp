/*
* main.cpp
* Reads in data and runs 2D particle filter.
*  Created on: Dec 13, 2016
*      Author: Tiffany Huang
*/

#include "stdafx.h" // for Microsoft Visual Studio
#include <iostream>
#include <ctime>
#include <iomanip>
#include <random>

#include "particle_filter.h"
#include "helper_functions.h"

int main()
{

	int time_steps_before_lock_required = 100;	// number of time steps before accuracy is checked by grader
	double max_runtime = 45;					// max allowable runtime to pass (sec)
	double max_translation_error = 1;			// Max allowable translation error to pass (m)
	double max_yaw_error = 0.05;				// Max allowable yaw error (rad)

	int start = clock();						// start timer

	double delta_t = 0.1;
	double sensor_range = 50;

	double sigma_pos[3] = { 0.3, 0.3, 0.01 };	// GPS measurement uncertainty [x (m), y (m), theta (rad)]
	double sigma_landmark[2] = { 0.3, 0.3 };	// Landmark measurement uncertainty [x (m), y (m)]

	// for noise generation
	std::default_random_engine gen;
	std::normal_distribution<double> N_x_init(0, sigma_pos[0]);
	std::normal_distribution<double> N_y_init(0, sigma_pos[1]);
	std::normal_distribution<double> N_theta_init(0, sigma_pos[2]);
	std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
	std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
	double n_x, n_y, n_theta, n_range, n_heading;

	// read map data
	// get the coordianes of each landmark in the global coodinate system
	Map map;
	if (!read_map_data("../data/map_data.txt", map)) {
		std::cout << "Error: Could not open map file" << std::endl;
		return -1;
	}

	// read position data
	std::vector<control_s> position_meas;
	if (!read_control_data("data/control_data.txt", position_meas)) {
		std::cout << "Error: Could not open position/control measurement file" << std::endl;
		return -1;
	}

	// read ground truth data
	std::vector<ground_truth> gt;
	if (!read_gt_data("data/gt_data.txt", gt)) {
		std::cout << "Error: Could not open ground truth data file" << std::endl;
		return -1;
	}

	// run particle filter
	int num_time_steps = position_meas.size();
	ParticleFilter pf;
	double total_error[3] = { 0, 0, 0};
	double cum_mean_error[3] = { 0, 0, 0 };

	for (int i(0); i < num_time_steps; ++i) {
		
		std::cout << "Time step: " << i << std::endl;
		std::ostringstream file;
		file << "data/observation/observations_" << std::setfill('0') << std::setw(6) << i + 1 << ".txt";
		
		// read in landmarks observations for current time step
		std::vector<LandmarkObs> observations;
		if (!read_landmark_data(file.str(),  observations)) {
			std::cout << "Error: Could not open observation file " << i+1 << std::endl;
			return -1;
		}

		if (!pf.initialized()) {
			n_x = N_x_init(gen);
			n_y = N_y_init(gen);
			n_theta = N_theta_init(gen);
			pf.init(gt[i].x + n_x, gt[i].y + n_y, gt[i].theta + n_theta, sigma_pos);
		}
		else {
			// predict vehicle next step (noiseless)
			pf.prediction(delta_t, sigma_pos, position_meas[i - 1].velocity, position_meas[i - 1].yawrate);
		}

		// simulate addition of noise to noiseless observation data
		std::vector<LandmarkObs> noisy_observations;
		LandmarkObs obs;
		for (int j(0); j < observations.size(); ++j) {
			n_x = N_obs_x(gen);
			n_y = N_obs_y(gen);
			obs = observations[j];
			obs.x = obs.x + n_x;
			obs.y = obs.y + n_y;
			noisy_observations.push_back(obs);
		}

		// update weights and resample
		pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map);
		pf.resample();

		std::vector<Particle> particles = pf.particles;
		int num_particles = particles.size();
		double highest_weight = 0.0;
		Particle best_particle;
		for (int j(0); j < num_particles; ++j) {
			if (particles[j].weight > highest_weight) {
				highest_weight = particles[j].weight;
				best_particle = particles[j];
			}
		}
		double *avg_error = getError(gt[i].x, gt[i].y, gt[i].theta,
									 best_particle.x, best_particle.y, best_particle.theta);

		for (int j(0); j < 3; ++j) {
			total_error[j] += avg_error[j];
			cum_mean_error[j] = total_error[j] / (double)(i + 1);
		}

		std::cout << "Cumulative mean weighted error: x " << cum_mean_error[0] << " y "
				  << cum_mean_error[1] << " yaw " << cum_mean_error[2] << std::endl;
		
		if (i >= time_steps_before_lock_required) {
			if (cum_mean_error[0] > max_translation_error ||
				cum_mean_error[1] > max_translation_error ||
				cum_mean_error[2] > max_translation_error) {
				
				if (cum_mean_error[0] > max_translation_error) {
					std::cout << "Your x error, " << cum_mean_error[0]
						<< " is larger than the maximum allowable error "
						<< max_translation_error << std::endl;
				}
				else if (cum_mean_error[1] > max_translation_error) {
					std::cout << "Your y error, " << cum_mean_error[1]
						<< " is larger than the maximum allowable error "
						<< max_translation_error << std::endl;
				}
				else {
					std::cout << "Your yaw error, " << cum_mean_error[2]
						<< " is larger than the maximum allowable error "
						<< max_translation_error << std::endl;
				}
				return -1;
			}
		}
	}

	int stop = clock();
	double runtime = (stop - start) / double(CLOCKS_PER_SEC);
	std::cout << "Runtime (sec): " << runtime << std::endl;

	if (runtime < max_runtime && pf.initialized()) {
		std::cout << "Success! Your particle filter passed!" << std::endl;
	}
	else if (!pf.initialized()) {
		std::cout << "This is the starter code. You haven't initialized your filter." << std::endl;
	}
	else {
		std::cout << "Your runtime " << runtime << " is larger than the maximum allowable runtime, "
				  << max_runtime << std::endl;
		return -1;
	}

    return 0;
}

