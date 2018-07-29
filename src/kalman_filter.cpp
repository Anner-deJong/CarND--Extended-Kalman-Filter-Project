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
  // update, specific to lasar (regular Kalman Filter)
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  // update, generic part (other than that H and R are already set for laser)
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate, generic
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // update, specific to radar (extended Kalman Filter)
  // BEWARE TO NORMALIZE phi between -pi and pi
  float p_x = x_[0];
  float p_y = x_[1];
  float v_x = x_[2];
  float v_y = x_[3];
  
  float rho = sqrt(p_x*p_x + p_y*p_y);
  float phi = atan2(p_y, p_x);
  float rho_dot = (p_x * v_x + p_y * v_y)/ rho;
  
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;
  
  VectorXd y = z - z_pred;
  // this if statement can be optimized in a single one by using abs() and sign()
  // (not sure if that would actually make it faster)
  if (y[1] > M_PI) { //M_PI is pi, included via cmath
    y[1] -= 2 * M_PI;
  } else if (y[1] < -M_PI ) {
    y[1] += 2 * M_PI;
  }

  // update, generic part (other than that H and R are already set for Radar)
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate, generic
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
