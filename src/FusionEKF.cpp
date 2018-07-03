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

  previous_timestamp_ = 0;

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // laser measurement projection matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // radar measurement projection matrix
  Hj_ = MatrixXd(3, 4);
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  // initial transition matrix
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // initial state covariance matrix
  ekf_.P_ = MatrixXd(4,4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

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

    // Initialize state with first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // Convert radar from polar to cartesian coordinates and initialize state.
      float rho =     measurement_pack.raw_measurements_(0);
      float theta =   measurement_pack.raw_measurements_(1);
      float rhodot =  measurement_pack.raw_measurements_(2);

      ekf_.x_ << rho * cos(theta), rho * sin(theta), rhodot * cos(theta), rhodot * sin(theta);
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      // Initialize with px, py, and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_(0),measurement_pack.raw_measurements_(1), 0, 0;
    }

    // mark timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  
  // Update the time (measured in seconds)
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // update state transition matrix with integrated time
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // update process covariance matrix
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 0.25*dt4*noise_ax, 0, 0.5*dt3*noise_ax, 0,
             0, 0.25*dt4*noise_ay, 0, 0.5*dt3*noise_ay,
             0.5*dt3*noise_ax, 0, dt2*noise_ax, 0,
             0, 0.5*dt3*noise_ay, 0, dt2*noise_ay;

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
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //std::cout << "x_ = " << ekf_.x_ << std::endl;
  //std::cout << "P_ = " << ekf_.P_ << std::endl;
}
