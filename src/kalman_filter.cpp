#include "kalman_filter.h"
#include <cmath>


using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  
  // update specific to laser (regular Kalman Filter)
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  // update common part
  _UpdateCommon(y); 
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  // update specific to radar (extended Kalman Filter)
  // BEWARE TO NORMALIZE phi between -pi and pi
  float p_x = x_[0];
  float p_y = x_[1];
  float v_x = x_[2];
  float v_y = x_[3];
  
  // comment from Udacity: rho could be zero, for which case phi and rho_dot need to be robust
  float rho = sqrt(p_x*p_x + p_y*p_y);
  float phi = (rho>0) ? atan2(p_y, p_x) : 0;
  float rho_dot = (p_x * v_x + p_y * v_y)/ std::max(rho, float(0.00001));
  
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  
  VectorXd y = z - z_pred;
  // OLD VERSION: this if statement can be optimized in a single one by using abs() and sign()
  // (not sure if that would actually make it faster)
  // UPDATE: comment from Udacity: instead of unreadable if-else statement use dedicated simple function
  _NormalizeTanAngle(y(1));
  
  // update common part
  _UpdateCommon(y);
  
}

void KalmanFilter::_NormalizeTanAngle(double& phi) {
  phi = atan2(sin(phi), cos(phi));
  }

void KalmanFilter::_UpdateCommon(const VectorXd &y) {
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht + R_;
  MatrixXd Si = S_.inverse();
  MatrixXd K_ = P_ * Ht * Si;
  
  // new estimate
  x_ = x_ + (K_ * y);
  // comment from Udacity: The following line can be simplified
  // P_ = (I - K * H_) * P_; // Identity matrix I no longer declared
  P_ -= K_ * H_ * P_;
  
}
