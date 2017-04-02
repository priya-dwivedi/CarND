#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include<math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //set the acceleration noise components
 
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //Laser H matrix
  H_laser_ << 1,0,0,0,
        	0,1,0,0;
		  

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
  	
  	cout << "Kalman Filter Initialization " << endl;

	//Initialize the state ekf_.x_
	ekf_.x_ = VectorXd(4);
	ekf_.x_ << 1, 1, 1, 1;
	
	// Initialize State covariance matrix P
  	ekf_.P_ = MatrixXd(4, 4);
  	ekf_.P_<< 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
	
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
     float px;
     float py;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    	
    	 /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      
      px = rho * cos(phi);
      py = rho * sin(phi);
     
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    	
    	//initialize with position from file
      px = measurement_pack.raw_measurements_[0];
	  py = measurement_pack.raw_measurements_[1];
    }

    // Check if pc, py are very small
    if(fabs(px) < 0.0001){
        px = 0.1;
        cout << "init px too small" << endl;
    }

    if(fabs(py) < 0.0001){
        py = 0.1;
        cout << "init py too small" << endl;
    }
    
    //Initialize with zero velocity
    ekf_.x_ << px,py,0,0;
    cout << "EKF: " << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   //compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//Modify the F matrix so that the time is integrated
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, dt, 0,
		  0, 1, 0, dt,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	
	ekf_.Q_ = MatrixXd(4, 4);
	float noise_ax = 9;
    float noise_ay = 9;
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  

   ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.hx_ = VectorXd(3);
      
    float px = ekf_.x_[0];
    float py = ekf_.x_[1];
    float vx = ekf_.x_[2];
    float vy = ekf_.x_[3];
    
    float rho;
    float phi;
    float rhodot;
	
	if(fabs(px) < 0.0001 or fabs(py) < 0.0001){
        
        if(fabs(px) < 0.0001){
          px = 0.0001; //set px to 0.0001
        }

        if(fabs(py) < 0.0001){
          py = 0.0001;   //set py to 0.0001
        }
        
        rho = sqrt(px*px + py*py);
        phi = 0;	//set phi to zero
        rhodot = 0;	//set vel to zero
  
      } else {
        rho = sqrt(px*px + py*py);
        phi = atan2(py,px); 
        rhodot = (px*vx + py*vy) /rho;
      }      

	
	ekf_.hx_ <<rho, phi, rhodot;
	ekf_.R_ = R_radar_;
     ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
    else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
