/*
* particle_filter.cpp
*
*  Created on: Dec 12, 2016
*      Author: Tiffany Huang
*
*  Modified on: May 23, 2017
*      Author: Uriel Tallon
*/

//#include "stdafx.h"
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std_var[]) {
	// set number of particles
	// initialize all particles to first positions and weights to 1
	// add random gaussian noise to each particle

	num_particles = 50;
	weights.resize(num_particles);
	
	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(0.0, std_var[0]);
	std::normal_distribution<double> dist_y(0.0, std_var[1]);
	std::normal_distribution<double> dist_theta(0.0, std_var[2]);

	for (size_t i(0); i < num_particles; ++i) {
		Particle p_init;
		p_init.id = i;
		p_init.x = x + dist_x(gen);
		p_init.y = y + dist_y(gen);
		p_init.theta = theta + dist_theta(gen);
		p_init.weight = 1.0f;
		weights[i] = 1.0f;
		particles.push_back(p_init);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// add measurement to each particle
	// add random gaussian noise

	std::default_random_engine gen;

	for (int i(0); i < num_particles; ++i) {
		
		double theta = particles[i].theta;

		std::normal_distribution<double> dist_x(0.0, std_pos[0]);
		std::normal_distribution<double> dist_y(0.0, std_pos[1]);
		std::normal_distribution<double> dist_theta(0.0, std_pos[2]);

		// same problematic as kalman filter, need to distinguish the cases
		// where the yaw rate is null to prevent dividing by zero
		if (fabs(yaw_rate) < 0.001) {
			particles[i].x += velocity * delta_t * cos(theta) + dist_x(gen);
			particles[i].y += velocity * delta_t * sin(theta) + dist_y(gen);
		}
		else {
			particles[i].x += (velocity / yaw_rate) * (sin(theta + yaw_rate * delta_t) - sin(theta)) + dist_x(gen);
			particles[i].y += (velocity / yaw_rate) * (-cos(theta + yaw_rate * delta_t) + cos(theta)) + dist_y(gen);
		}
		particles[i].theta += yaw_rate * delta_t + dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
									 std::vector<LandmarkObs> &observations) {
	// find predicted measurement that is closest to each observed measurement and assigne the
	// observed measurement to this particular landmark
	// UNUSED

	for (size_t i(0); i < observations.size(); ++i) {

		double minimal_distance(1000000.0); // need to start with a big badass number
		int best_id(-1); // temporary id
		// for each prediction, find the one that is
		// the closest to the landmark i
		for (size_t j(0); j < predicted.size(); ++j) {
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (distance < minimal_distance) {
				minimal_distance = distance;
				best_id = predicted[j].id;
			}
		}
		// associate the id of the closest prediction to the landmark
		observations[i].id = best_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
								   std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	
	weights.clear();

	for (int p_ind(0); p_ind < num_particles; ++p_ind) {
		
		std::vector<LandmarkObs> observations_global;
		double theta = particles[p_ind].theta;

		// translate the observations from the car coordinates to the global coordinates system
		// for this particle (prediction has already been called, thus using predicted value for theta)
		for (size_t i(0); i < observations.size(); ++i) {
			
			LandmarkObs obs_global;
			double obs_x = observations[i].x;
			double obs_y = observations[i].y;
			double theta = particles[p_ind].theta;

			// pity we don't have eigen here...
			obs_global.id = -1;
			obs_global.x = obs_x * cos(theta) - obs_y * sin(theta) + particles[p_ind].x;
			obs_global.y = obs_x * sin(theta) + obs_y * cos(theta) + particles[p_ind].y;

			observations_global.push_back(obs_global);
		}

		// find all landmarks within sensor range for this particle
		std::vector<LandmarkObs> in_range_landmarks;
		for (size_t i(0); i < map_landmarks.landmark_list.size(); ++i) {
			

			double distance_particle = dist(particles[p_ind].x, particles[p_ind].y,
							map_landmarks.landmark_list[i].x_f, map_landmarks.landmark_list[i].y_f);

			if (distance_particle <= sensor_range) {
				LandmarkObs landmark;
				landmark.id = map_landmarks.landmark_list[i].id_i;
				landmark.x = map_landmarks.landmark_list[i].x_f;
				landmark.y = map_landmarks.landmark_list[i].y_f;
				in_range_landmarks.push_back(landmark);
			}

		}
		
		// find the closest observations to each landmark and associate
		// said landmark to observation
		//dataAssociation(in_range_landmarks, observations_global);
		
		/*Try direct approach*/
		// find the best observation for each landmark
		double probability_total(1.0);
		for (size_t i(0); i < in_range_landmarks.size(); ++i) {
			double minimal_distance(1000000.0); // need to start with a big badass number
			int best_id(-1); // temporary id
			for (size_t j(0); j < observations_global.size(); ++j) {
				double distance = dist(observations_global[j].x, observations_global[j].y,
									   in_range_landmarks[i].x, in_range_landmarks[i].y);
				if (distance < minimal_distance) {
					minimal_distance = distance;
					best_id = j;
				}
			}
            // use the formula for a bivariate normal distribution
            // (assume correlation = 0.0)
			double op1 = -pow(in_range_landmarks[i].x - observations_global[best_id].x, 2) / (2.0 * pow(std_landmark[0], 2));
			double op2 = -pow(in_range_landmarks[i].y - observations_global[best_id].y, 2) / (2.0 * pow(std_landmark[1], 2));
			double probability = exp(op1 + op2) / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);
			probability_total *= probability;
		}
		weights.push_back(probability_total);
		particles[p_ind].weight = probability_total;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	
	// the distribution is generated using the weights previously updated
	std::default_random_engine gen;
	std::discrete_distribution<size_t> dist_weights(weights.begin(), weights.end());
	
	std::vector<Particle> rsamp;

	// draw particles according to their weights: the higher the
	// weight, the more likely it is for the particle to be drawn
	for (int i(0); i < num_particles; ++i) {
		Particle r_part = particles[dist_weights(gen)];
		rsamp.push_back(r_part);
	}
	particles = rsamp;
}

void ParticleFilter::write(std::string filename) {
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i(0); i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}