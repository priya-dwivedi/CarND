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
#include <map>
#include <math.h>
#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
		// Set the number of particles. 
		num_particles = 100;
			
		// TODO: Set standard deviations for x, y, and theta.
		double std_x = std[0];
		double std_y = std[1];
		double std_theta = std[2];

		
		// This line creates a normal (Gaussian) distribution for x - GPS position
		normal_distribution<double> dist_x(x, std_x);
		
		// TODO: Create normal distributions for y and theta - - GPS position
		normal_distribution<double> dist_y(y, std_y);
		
		normal_distribution<double> dist_theta(theta, std_theta);
		
		default_random_engine gen;
		
		//Initialize all particles to first position (based on estimates of 
		//   x, y, theta and their uncertainties from GPS) 
		// Add random Gaussian noise to each particle.
		
		for (int i = 0; i < num_particles; ++i) {
			double sample_x, sample_y, sample_theta;
			
			// Sample  and from these normal distrubtions like this: 
			sample_x = dist_x(gen);
			sample_y = dist_y(gen);
			sample_theta = dist_theta(gen);
			// where "gen" is the random engine initialized earlier
			
			Particle particle;
			particle.id = i;
			particle.x = sample_x;
			particle.y = sample_y;
			particle.theta = sample_theta;
			particle.weight = 1.0; //initialize with a weight 1.0
			
			particles.push_back(particle);
			weights.push_back(particle.weight); //initialize the weights vector
			
		}
		
		is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

		
	default_random_engine gen;
	
	// This line creates a normal (Gaussian) noise for x, y and theta around zero. 
	//Noise has been set propotional to the time step delta_t
	normal_distribution<double> dist_x(0, std_x*delta_t);
	normal_distribution<double> dist_y(0, std_y*delta_t);
	normal_distribution<double> dist_theta(0, std_theta*delta_t);

	
	for (int i = 0; i < num_particles; ++i) {
			
			Particle &particle = particles[i]; //create a pointer to the ith particle
			

			// Calculate x and y position and add noise 
			
			if (yaw_rate ==0)
			{
				cout<<"Zero Yaw Rate ";
			particle.x += velocity*delta_t*cos(particle.theta) + dist_x(gen);
			particle.y += velocity*delta_t*sin(particle.theta) + dist_y(gen);
			
			particle.theta= particle.theta + dist_theta(gen);
			}
			
			else //Non Zero Yaw Rate
			{
			particle.x += (velocity/yaw_rate)*(sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta)) + dist_x(gen);
			particle.y += (velocity/yaw_rate)*(cos(particle.theta) - cos(particle.theta + yaw_rate*delta_t)) + dist_y(gen);
			
			particle.theta += yaw_rate*delta_t + dist_theta(gen);
			}

			}

}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	
	// First we tranform each observation from particle coordinate system to vehicle coordinate system
	
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void data_association(std::vector<LandmarkObs>& predicted, const std::vector<LandmarkObs>& observations,
		const std::vector<LandmarkObs>& transformed_landmark_list, const Particle& particle){
	//Associate each observation to its mostly likely predicted landmark measurements for a particular particle
	
	//Run through every observation
	for (int i =0; i< observations.size();i++){
		const LandmarkObs &landmarkobs = observations[i];
		
		//if transformed_landmark_list is empty then cout that
		if (transformed_landmark_list.size() ==0)
		{
			cout<<"No transformed landmark";
		}

		//  Set closest distance as that betweent the ith observation and first prediction
		double closest_calc = (observations[i].x - transformed_landmark_list[0].x)*(observations[i].x - transformed_landmark_list[0].x) 
								+(observations[i].y - transformed_landmark_list[0].y)*(observations[i].y - transformed_landmark_list[0].y);
		double closest_dist = sqrt(closest_calc);
		int predicted_landmark_ind = 0; //Initialize the first prediction as the landmark ID
		//cycle through the list of transformed landmark
		for(int j=0; j< transformed_landmark_list.size(); j++){
			const LandmarkObs &landmark_chosen = transformed_landmark_list[j];

			double x_dist = landmark_chosen.x - landmarkobs.x;
			double y_dist = landmark_chosen.y - landmarkobs.y;
			double dist_total = sqrt(x_dist*x_dist + y_dist*y_dist);
			
			//if any is closer than closest distance then set that as the new closest landmark
			if(dist_total < closest_dist ){
				closest_dist = dist_total;
				predicted_landmark_ind = j;
			}
		}
			//For each observation store the closest landmark in predicted vector
			const LandmarkObs& landmark_closest = transformed_landmark_list[predicted_landmark_ind];
			//cout<<"Closest landmark is"<<predicted_landmark_ind;
			predicted.push_back(landmark_closest);
		
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

	for (int i = 0; i < num_particles; i++) {
		
		Particle &particle = particles[i];
		vector<LandmarkObs> transformed_landmark_list ;
		LandmarkObs landmark_obs;
		
		
		// For each landmark transform the landmark in the particle system and store in a transformed landmark list
		for (int j = 0; j< map_landmarks.landmark_list.size(); j++)
		{
			Map::single_landmark_s &landmark = map_landmarks.landmark_list[j];
			
		
				//transform this landmark in particle coordinate system. 
				double cos_theta = cos(particle.theta - M_PI / 2);
				double sin_theta = sin(particle.theta - M_PI / 2);
				
				landmark_obs.x = -(landmark.x_f - particle.x) * sin_theta + (landmark.y_f - particle.y) * cos_theta;
				landmark_obs.y = -(landmark.x_f - particle.x) * cos_theta - (landmark.y_f - particle.y) * sin_theta;

				landmark_obs.id = landmark.id_i;
				
				//And add to the a  new landmark list
				transformed_landmark_list.push_back(landmark_obs);
				
				//double dist_total = sqrt((landmark.x_f - particle.x)*(landmark.x_f - particle.x) +(landmark.y_f - particle.y)*(landmark.y_f - particle.y));
				//if (dist_total <= sensor_range)

		}
			
			// Now associate observations with the transformed landmark list  
			std::vector<LandmarkObs> predicted ;
			data_association(predicted, observations, transformed_landmark_list, particle);
			
		
		//Calculate multi-variate guassian on each observation and its closest landmark 
		
		double sigma_x = std_landmark[0];
		double sigma_y = std_landmark[1];
		long double guassian_prob = 0;

		
		for(int i=0; i< observations.size();i++){
		//Compute predicted landmark measurement for the particle
		const LandmarkObs &landmarkobs = observations[i];
		const LandmarkObs& landmarkpred = predicted[i];

		double gap_x = landmarkobs.x - landmarkpred.x;
		double gap_y = landmarkobs.y - landmarkpred.y;

		double denom = 1.0/(2 * M_PI * sigma_x * sigma_y);
		double term1 = pow(gap_x, 2)/pow(sigma_x, 2) + pow(gap_y,2)/pow(sigma_y, 2);
		double term2 = exp(-0.5 * term1);
		double weight = denom * term2;

		//multiply density for all predicted measurements
		//for first measurement set product as the first weight
		if(guassian_prob == 0){
			guassian_prob = weight;
		}else{
			guassian_prob = guassian_prob *weight;
		}
		
	}
	//Update particle weight
	particle.weight = guassian_prob;
	weights[i] = guassian_prob;
	
}
}


void ParticleFilter::resample() {
	

	std::vector<Particle> resampled_particles;
	
	//Define max_weight and cache it 
	double max_weight =  *max_element(std::begin(weights), std::end(weights));
	
	// Random number generator
	
    std::random_device rd;
    std::mt19937 gen(rd());
    
    //Uniform random distributions for index and 2*max weight
    std::uniform_int_distribution<int> dist_num_parts(0, num_particles);
    
   std::uniform_real_distribution<double> weights_gen(0, 2*max_weight);
   
   // Option to draw with some noise
	std::normal_distribution<double> noise_x(0,0.003);
	std::normal_distribution<double> noise_y(0,0.003);
	std::normal_distribution<double> noise_theta(0,0.0001);
	
	int index = dist_num_parts(gen);
	double beta = 0.0;
	
	
	for (int i =0; i< num_particles; i++)
	{
		beta += weights_gen(gen);
		while (beta > weights[index])
		{
			beta -= weights[index];
			index = (index +1)% num_particles;
			
		}
		Particle particle;
		particle.x = particles[index].x + noise_x(gen);
		particle.y = particles[index].y + noise_y(gen);
		particle.theta = particles[index].theta +noise_theta(gen);
		particle.weight = particles[index].weight;
		
		resampled_particles.push_back(particle);
	}
	 
	 particles = resampled_particles;
	
}


void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
