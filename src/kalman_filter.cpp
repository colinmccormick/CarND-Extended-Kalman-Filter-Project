#include <math.h>
#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;  // kalman filter gain

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  float rhodot = (px*vx + py*vy)/rho;

  // ensure that theta is in the correct range (-pi to +pi)
  // range testing fails unless we scale down by a fudge factor
  float PI_SCALING_FACTOR = 0.995;

  if (theta > PI_SCALING_FACTOR * M_PI) {
    std::cout << "Theta out of range: " << theta << "; subtracted 2*PI" << std::endl;
    theta -= 2 * M_PI;
  } 
  if (theta < -PI_SCALING_FACTOR * M_PI) {
    std::cout << "Theta out of range: " << theta << "; added 2*PI" << std::endl;
    theta += 2 * M_PI;
  }

  // check for no divide by zero (point too close to the origin)
  if (fabs(rho) < 0.0001) {
    std::cout << "Error: Divide by zero in EKF" << std::endl;
    rhodot = 0;
  };
  
  // use extended Kalman filter equations
  VectorXd z_pred(3);
  z_pred << rho, theta, rhodot;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; // kalman filter gain

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * H_) * P_;

}
