#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.3 ;//Initial value set -1.4

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;  //Initial value set -0.7

  // Laser measurement noise standard deviation position1 in m
  //Note measurement noise information is provided by manufacturer and should not be updated
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  //Initial initialization
  is_initialized_ = false;

  //initial time step
  previous_timestamp_ = 0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	if (!is_initialized_) {

	  	cout << "Unscented Kalman Filter Initialization " << endl;

		//Initialize the state x_
		x_ << 1, 1, 1, 1, 1;

		// Initialize State covariance matrix P using values suggested in the class
		
		P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
		          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
		           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
		          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
		          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
		
		
		//P_.fill(0.0);  - Tried a zero initialization as well
		
	    // first measurement
	     float px;
	     float py;
	     float v;
	     float yaw;
	     float yaw_rt;

	    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

	    	 /**
	      Convert radar from polar to cartesian coordinates and initialize state.
	      */

	      float rho = meas_package.raw_measurements_(0);
	      float phi = meas_package.raw_measurements_(1);
	      float rhodot = meas_package.raw_measurements_(2);

	      px = rho * cos(phi);
	      py = rho * sin(phi);

	      //Initialize with zero but check later if radar data can be used here
	      v = 0;
	      yaw = 0;
	      yaw_rt = 0;


	    }
	    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

	    	//initialize with position from file
	      px = meas_package.raw_measurements_[0];
		  py = meas_package.raw_measurements_[1];
		  v = 0;
		  yaw = 0;
		  yaw_rt = 0;
	    }

	    // Check if px, py are very small
	    if(fabs(px) < 0.0001){
	        px = 0.01;
	        cout << "init px too small" << endl;
	    }

	    if(fabs(py) < 0.0001){
	        py = 0.01;
	        cout << "init py too small" << endl;
	    }


	    //Initialize
	    x_ << px,py,v,yaw, yaw_rt;
	    cout << "EKF: " << endl;
	    previous_timestamp_ = meas_package.timestamp_;
	    is_initialized_ = true;
	    return;
	}

	//Else once initialized generate sigma points

	//set state dimension
	  int n_x = 5;

	 //set augmented dimension
	  int n_aug = 7;

	  //define spreading parameter
	  double lambda = 3 - n_aug;

	  //create augmented mean vector
	  VectorXd x_aug = VectorXd(n_aug);

	  //create augmented state covariance
	  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

	  //create sigma point matrix
	  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);


	  //create augmented mean state
	  x_aug.head(5) = x_;
	  x_aug(5) = 0;
	  x_aug(6) = 0;

	  //create augmented covariance matrix
	  P_aug.fill(0.0);
	  P_aug.topLeftCorner(5,5) = P_;
	  P_aug(5,5) = std_a_*std_a_;
	  P_aug(6,6) = std_yawdd_*std_yawdd_;

	  //create square root matrix
	  MatrixXd L = P_aug.llt().matrixL();

	  //create augmented sigma points
	  Xsig_aug.col(0)  = x_aug;
	  for (int i = 0; i< n_aug; i++)
	  {
	    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
	    Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
	  }

		//Measurement update
	   //create matrix with predicted sigma points as columns
	   MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

	  	  //compute the time elapsed between the current and previous measurements
	  float delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	   previous_timestamp_ = meas_package.timestamp_;

	   //predict sigma points
	   	  for (int i = 0; i< 2*n_aug+1; i++)
	   	  {
	   	    //extract values for better readability
	   	    double p_x = Xsig_aug(0,i);
	   	    double p_y = Xsig_aug(1,i);
	   	    double v = Xsig_aug(2,i);
	   	    double yaw = Xsig_aug(3,i);
	   	    double yawd = Xsig_aug(4,i);
	   	    double nu_a = Xsig_aug(5,i);
	   	    double nu_yawdd = Xsig_aug(6,i);

	   	    //predicted state values
	   	    double px_p, py_p;

	   	    //avoid division by zero
	   	    if (fabs(yawd) > 0.001) {
	   	        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
	   	        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
	   	    }
	   	    else {
	   	        px_p = p_x + v*delta_t*cos(yaw);
	   	        py_p = p_y + v*delta_t*sin(yaw);
	   	    }

	   	    double v_p = v;
	   	    double yaw_p = yaw + yawd*delta_t;
	   	    double yawd_p = yawd;

	   	    //add noise
	   	    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
	   	    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
	   	    v_p = v_p + nu_a*delta_t;

	   	    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
	   	    yawd_p = yawd_p + nu_yawdd*delta_t;

	   	    //write predicted sigma point into right column
	   	    Xsig_pred(0,i) = px_p;
	   	    Xsig_pred(1,i) = py_p;
	   	    Xsig_pred(2,i) = v_p;
	   	    Xsig_pred(3,i) = yaw_p;
	   	    Xsig_pred(4,i) = yawd_p;

	   	  }


	  //create vector for weights
	  VectorXd weights = VectorXd(2*n_aug+1);

	  //create vector for predicted state
	  VectorXd x = VectorXd(n_x);

	  //create covariance matrix for prediction
	  MatrixXd P = MatrixXd(n_x, n_x);


	  // set weights
	  double weight_0 = lambda/(lambda+n_aug);
	  weights(0) = weight_0;
	  for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
	    double weight = 0.5/(n_aug+lambda);
	    weights(i) = weight;
	  }

	  //predicted state mean
	  x.fill(0.0);
	  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points
	    x = x+ weights(i) * Xsig_pred.col(i);
	  }

	  //predicted state covariance matrix
	  P.fill(0.0);
	  for (int i = 0; i < 2 * n_aug + 1; i++) {  //iterate over sigma points

	  // state difference
	  VectorXd x_diff = Xsig_pred.col(i) - x;
	  //angle normalization
	  while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
	  while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

	  P = P + weights(i) * x_diff * x_diff.transpose() ;

	  //Set to our variables

	  x_ = x;
	  P_ = P;


	//Update LIDAR measurements
	
	if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
	
		if (use_laser_ == true)
		{
			H_= MatrixXd(2, 5);

			//Laser H matrix
			H_ << 1,0,0,0,0,
				  0,1,0,0,0;
			
			//Sensor data

			float px = meas_package.raw_measurements_(0);
			float py = meas_package.raw_measurements_(1);


			 VectorXd z = VectorXd(2);
			 z << px, py;
			 
		    MatrixXd R_laser = MatrixXd(2,2);
			R_laser <<    std_laspx_*std_laspx_, 0, 
					0, std_laspy_*std_laspy_; 
				

			VectorXd z_pred = H_ * x_;
			VectorXd y = z - z_pred;
			MatrixXd Ht = H_.transpose();
			MatrixXd S = H_ * P_ * Ht + R_laser;
			MatrixXd Si = S.inverse();
			MatrixXd PHt = P_ * Ht;
			MatrixXd K = PHt * Si;

			//new estimate
			x_ = x_ + (K * y);
			long x_size = x_.size();
			MatrixXd I = MatrixXd::Identity(x_size, x_size);
			P_ = (I - K * H_) * P_;
			
			//calculate NIS LIDAR
			
		    NIS_laser_ = (meas_package.raw_measurements_-z_pred).transpose()*S.inverse()*(meas_package.raw_measurements_-z_pred);

			//NIS_laser_ = y.transpose()*S.inverse()*y;
		}
	}
	
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
	{

		if (use_radar_ == true)

		{
		  //set measurement dimension, radar can measure r, phi, and r_dot
		  int n_z = 3;

		  //define spreading parameter
		  double lambda = 3 - n_aug;

		  //set vector for weights
		  VectorXd weights = VectorXd(2*n_aug+1);
		   double weight_0 = lambda/(lambda+n_aug);
		  weights(0) = weight_0;
		  for (int i=1; i<2*n_aug+1; i++) {
			double weight = 0.5/(n_aug+lambda);
			weights(i) = weight;
		  }

		  //create matrix for sigma points in measurement space
		  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

		  //transform sigma points into measurement space
			for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

			  // extract values for better readibility
			  double p_x = Xsig_pred(0,i);
			  double p_y = Xsig_pred(1,i);
			  double v  = Xsig_pred(2,i);
			  double yaw = Xsig_pred(3,i);

			  double v1 = cos(yaw)*v;
			  double v2 = sin(yaw)*v;

			  // measurement model
			  Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
			  Zsig(1,i) = atan2(p_y,p_x);                                 //phi
			  Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
			}

			//mean predicted measurement
			VectorXd z_pred = VectorXd(n_z);
			z_pred.fill(0.0);
			for (int i=0; i < 2*n_aug+1; i++) {
				z_pred = z_pred + weights(i) * Zsig.col(i);
			}

			//measurement covariance matrix S
			MatrixXd S = MatrixXd(n_z,n_z);
			S.fill(0.0);
			for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
			  //residual
			  VectorXd z_diff = Zsig.col(i) - z_pred;

			  //angle normalization
			  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
			  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

			  S = S + weights(i) * z_diff * z_diff.transpose();
			}

			//add measurement noise covariance matrix
			MatrixXd R = MatrixXd(n_z,n_z);
			R <<    std_radr_*std_radr_, 0, 0,
					0, std_radphi_*std_radphi_, 0,
					0, 0,std_radrd_*std_radrd_;
			S = S + R;

			//Update x_ and P_


			//create matrix for cross correlation Tc
			MatrixXd Tc = MatrixXd(n_x, n_z);

			//Sensor data

			float rho = meas_package.raw_measurements_(0);
			float phi = meas_package.raw_measurements_(1);
			float rhodot = meas_package.raw_measurements_(2);


			 VectorXd z = VectorXd(n_z);
			 z << rho, phi,rhodot;

			 //calculate cross correlation matrix
			 Tc.fill(0.0);
			 for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

			 //residual
			 VectorXd z_diff = Zsig.col(i) - z_pred;
			 //angle normalization
			 while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
			 while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

			 // state difference
			 VectorXd x_diff = Xsig_pred.col(i) - x;
			 //angle normalization
			 while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
			 while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

			 Tc = Tc + weights(i) * x_diff * z_diff.transpose();
			 }

			//Kalman gain K;
			MatrixXd K = Tc * S.inverse();

			//residual
			VectorXd z_diff = z - z_pred;

			//angle normalization
			while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
			while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		   //update state mean and covariance matrix
		   x_ = x_ + K * z_diff;
		   P_ = P_ - K*S*K.transpose();
		   
		   //calculate NIS RADAR
		   
		   NIS_radar_ = (meas_package.raw_measurements_-z_pred).transpose()*S.inverse()*(meas_package.raw_measurements_-z_pred);

			//NIS_radar_ = z_diff.transpose()*S.inverse()*z_diff;

		}
	}
	   
}
}
