/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <limits>
#include <map>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	num_particles = 30;

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	default_random_engine gen;
	particles.clear();
	for (int i=0; i<num_particles; ++i){
		Particle p;
		p.id=i;
		p.x=dist_x(gen);
		p.y=dist_y(gen);
		p.theta=dist_theta(gen);
		p.weight=1.0;
		particles.emplace_back(p);
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;  

	// Run prediction for all particles
	for (auto& p : particles) {
		// Add Gaussian noise to x, y, theta
		normal_distribution<double> dist_x(p.x, std_pos[0]);
		normal_distribution<double> dist_y(p.y, std_pos[1]);
		normal_distribution<double> dist_theta(p.theta, std_pos[2]);
		double x = dist_x(gen);
		double y = dist_y(gen);
		double theta = dist_theta(gen);

		// Motion model (bicycle model)
		double theta_f;
		double x_f;
		double y_f;
		if (fabs(yaw_rate) < 1e-6) {  // switching condition if yaw_rate close to zero, prevent division by zero
			theta_f = theta;
			x_f = x + velocity * delta_t * cos(theta);
			y_f = y + velocity * delta_t * sin(theta);
		} else {  // if yaw_rate != 0
			theta_f = theta + yaw_rate * delta_t;
			x_f = x + (velocity/yaw_rate) * (sin(theta_f) - sin(theta));
			y_f = y + (velocity/yaw_rate) * (cos(theta) - cos(theta_f));
		}

		// Update values in particle
		p.x = x_f;
		p.y = y_f;
		p.theta = theta_f;
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> landmarks, std::vector<LandmarkObs>& observations, double sensor_range) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// constant for nearest neighbor search radius percentage relative to sensor range
	const double kRadiusPercent = 0.10;

	// set all unmatched observation to -1
	for (auto& obs : observations) {
		obs.id = -1;
	}
	
	// search nearest neighbor map landmark for each observation
	map<int,map<int,double>> map_observation;

	// looping through all the landmark and observation pairs and storing the // 
	// (landmark_id,distance) pairs for each observation 
	for (unsigned int i=0; i<observations.size(); ++i){
		double d;
		LandmarkObs& obs=observations[i]; 
		for (const auto& landmark : landmarks){
			d = dist(landmark.x,landmark.y,obs.x,obs.y);
			if (d < kRadiusPercent*sensor_range)		// only take candidates within a search radius into consideration 
				map_observation[i].insert(std::pair<int,double>(landmark.id,d));
		}
	}
	for (const auto& kv : map_observation) {
		int obs_idx = kv.first;
		const auto& map_landmark = kv.second;
		vector<double> dist_values;
		vector<int> id_values;
		// loop through all landmark candidates and generate two vetors for distance and landmark id
		for (const auto& kv2 : map_landmark) {
			int landmark_idx = kv2.first;
			double distance = kv2.second;
			id_values.emplace_back(landmark_idx);
			dist_values.emplace_back(distance);
		}
		// find the index of the smallest distance
		auto it = min_element(dist_values.begin(), dist_values.end());
		int idx = distance(dist_values.begin(),it);
		// set the id of current observation to the landmark id with smallest distance
		observations[obs_idx].id = id_values[idx];
		std::cout<<"obs_idx: "<<obs_idx<<std::endl;
		std::cout<<"distances: ";
		for (const auto& e : dist_values)
			std::cout<<e<<",";
		std::cout<<std::endl;
		std::cout<<"min idx: "<<idx<<std::endl;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// set standard deviation for landmark measurement uncertainty
	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];

	// generate vector of LandmarkObs object to store map landmarks
	vector<LandmarkObs> landmarks_map_coord;
	landmarks_map_coord.reserve(map_landmarks.landmark_list.size());
	for (const auto& landmark : map_landmarks.landmark_list){
		LandmarkObs landmarkobs_new;
		landmarkobs_new.id = landmark.id_i;
		landmarkobs_new.x = landmark.x_f;
		landmarkobs_new.y = landmark.y_f;
		landmarks_map_coord.emplace_back(landmarkobs_new);
	}

	weights.clear();

	for (auto& p : particles){
		// transform sensor observations from vehicle coord to map coord
		vector<LandmarkObs> observations_map_coord;
		observations_map_coord.reserve(observations.size());
		for (const auto& obs : observations){
			LandmarkObs obs_trans = transVehicleFrameToMapFrame(p, obs);
			observations_map_coord.emplace_back(obs_trans);
		}

		// perform nearest neighbor association for sensor observations in map coord
		dataAssociation(landmarks_map_coord, observations_map_coord, sensor_range);

		double weight = 1.0;		// initial weight
		// declare variables for storing association and sensor measurements for visualization purpose
		vector<int> association;	
		vector<double> sense_x, sense_y;
		for (const auto& obs : observations_map_coord){
			if (obs.id != -1){	// if there is a nearest matched landmark
				
				int idx = obs.id;		//id refers to the index of the associated map landmark

				// storing association and sensor measurements
				association.emplace_back(idx);
				sense_x.emplace_back(obs.x);
				sense_y.emplace_back(obs.y);

				// update weight using bivariate Gaussian distribution
				double mu_x = landmarks_map_coord[idx].x;
				double mu_y = landmarks_map_coord[idx].y;
				double exponent = 0.5*(pow(obs.x-mu_x,2)/(sig_x * sig_x)+pow(obs.y-mu_y,2)/(sig_y * sig_y));
				double w = exp(-exponent) / (2 * M_PI * sig_x * sig_y);
				if (w<1e-6)		// ensure weights do not vanish to zero
					w=1e-6;
				weight*=w;

			}
		}
		p.weight = weight;
		weights.emplace_back(weight);
		// update association and sensor measurements for visualization
		SetAssociations(p,association,sense_x,sense_y);		
	}
	// normalize weights
	double w_normalizer = 0.0;
	for (const auto& w : weights){
		w_normalizer+=w;
	}
	for (unsigned int i=0; i<weights.size(); ++i){
		weights[i]/=w_normalizer;
		particles[i].weight /= w_normalizer; 
	}
	
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> new_particles; // new particle array

	// discrete_distribution for sampling particles with normalized weights
	default_random_engine generator;  
	discrete_distribution<int> distribution(weights.begin(), weights.end());  

	// Sample from discrete distribution
	for (unsigned int i=0; i < particles.size(); i++) {
		int random_idx = distribution(generator);

		new_particles.emplace_back(particles[random_idx]);
	}

	// Replace original particle vector
	particles = new_particles; 
}

LandmarkObs ParticleFilter::transVehicleFrameToMapFrame(Particle p, LandmarkObs obs){
	LandmarkObs obs_trans;
	obs_trans.id = obs.id;
	obs_trans.x = p.x + cos(p.theta)*obs.x - sin(p.theta)*obs.y;
	obs_trans.y = p.y + sin(p.theta)*obs.x + cos(p.theta)*obs.y;
	return obs_trans;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
