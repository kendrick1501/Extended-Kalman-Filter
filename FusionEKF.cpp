#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0.;

    //initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0,      0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0,      0,
                0,    0.0009, 0,
                0,    0,      0.09;

    //initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1,0,1,0,
               0,1,0,1,
               0,0,1,0,
               0,0,0,1;

    //initial state covariance matrix
    ekf_.P_ = MatrixXd(4,4);
    ekf_.P_ << 1,0,0,0,
               0,1,0,0,
               0,0,1000,0,
               0,0,0,1000;

    //initial process covariance matrix
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << 1,0,0,0,
               0,1,0,0,
               0,0,1,0,
               0,0,0,1;

    //set the acceleration noise components
    noise_ax = 9.0;
    noise_ay = 9.0;
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

        // first measurement
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 0, 0, 0, 0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
          /**
          Convert radar from polar to cartesian coordinates and initialize state.
          */

          // rho_measured, phi_measured, rhodot_measured
          float rho_ = measurement_pack.raw_measurements_[0];
          float phi_ = measurement_pack.raw_measurements_[1];
          float px = rho_*cos(phi_);
          float py = rho_*sin(phi_);

          ekf_.x_(0) = px;
          ekf_.x_(1) = py;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
          /**
          Initialize state.
          */
          ekf_.x_(0) = measurement_pack.raw_measurements_[0];
          ekf_.x_(1) = measurement_pack.raw_measurements_[1];
        }

        previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
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
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	//set the process covariance matrix
	ekf_.Q_ << dt_4/4*noise_ax, 0,               dt_3/2*noise_ax, 0,
			   0,               dt_4/4*noise_ay, 0,               dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0,               dt_2*noise_ax,   0,
			   0,               dt_3/2*noise_ay, 0,               dt_2*noise_ay;

    ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //Radar updates
    ekf_.R_ = MatrixXd(3, 3);
    ekf_.R_ = R_radar_;

    //Measurement matrix
    ekf_.H_ = MatrixXd(3, 4);
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    //Radar updates
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ = R_laser_;

    //Measurement matrix
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ << 1,0,0,0,
               0,1,0,0;

    ekf_.Update(measurement_pack.raw_measurements_);
    //cout << "Update: "<< ekf_.x_ << endl;

  }
}
