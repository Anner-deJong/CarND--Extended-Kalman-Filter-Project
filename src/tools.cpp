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
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  if (estimations.size() != ground_truth.size()) {
    cout << "CalculateRMSE() - Error - estimation size != ground truth size" << endl;
    return rmse;
  }
  // assuming all entries in either array have the same number of elements
  // thus we only need to check if the first entries of both arrays have equal size:
  float en = estimations[0].size(); // number of elements (n) in current (i) estimation
  float gn = ground_truth[0].size(); // number of elements (n) in current (i) ground truth
        
  if (en != gn) {
    cout << "CalculateRMSE() - Error - estimation[0] size != ground truth[0] size" << endl;
    return rmse;
  }
	
  //accumulate squared residuals
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
        
    // from answer, one less for-loop
    VectorXd difference = estimations[i] - ground_truth[i];
    VectorXd residual   = difference.array().square();
    rmse += residual;
  }
    
  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse  = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state, const MatrixXd& prev_Hj_) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj_(3,4);
  //get state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // check division by zero. If so, return previous Hj_
  // Make thus sure Hj_ is always valid! (initialize with zeros)
  if (px==0 && py==0) {
    cout << "Warning in CalculateJocobian () - Division by Zero" << endl;
    return Hj_;
  }
	
  //compute the Jacobian matrix

  float sq_rt2 = px*px + py*py;
  float sq_rt1 = sqrt(sq_rt2);
  float sq_rt3 = sq_rt2*sq_rt1;
  
  Hj_ << px/sq_rt1, py/sq_rt1, 0, 0,
        -py/sq_rt2, px/sq_rt2, 0, 0,
         py*(vx*py-vy*px)/sq_rt3, px*(vy*px-vx*py)/sq_rt3, px/sq_rt1,  py/sq_rt1;

  return Hj_;
}
