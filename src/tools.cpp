#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  // Declare rmse; size 4 is to hold values for px,py,vx,vy
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Check that estimations and ground_truth are non-zero and equal length
  if ( (estimations.size() != ground_truth.size()) || (estimations.size() == 0) ) {
  	std::cout << "Error on size of estimations or ground truth for RMSE." << std::endl;
  	return rmse;
  }

  // Calculate residuals, square, and accumulate in rmse
  for (int i=0; i < estimations.size(); ++i) {
  	VectorXd residuals = estimations[i] - ground_truth[i];
  	residuals = residuals.array() * residuals.array();
  	rmse += residuals;
  }

  // Calculate mean and square root
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  // Declare Jacobian variable and unpack state variables
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Compute useful values
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  // Check for extremely small denominators
  if ( fabs(c1) < 0.0001 ) {
  	std::cout << "Error: Zero x,y position in Jacobian calculation." << std::endl;
  	return Hj;
  }

  // Compute Jacobian
  Hj << (px/c2), (py/c2), 0, 0,
  		-(py/c1), (px/c1), 0, 0,
  		py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, (px/c2), (py/c2);

  return Hj; 

}
