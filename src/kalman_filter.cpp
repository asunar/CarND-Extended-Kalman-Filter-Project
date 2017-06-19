#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_ ; //lesson 5, section 8
  MatrixXd Ft = F_.transpose();

  P_ = F_ * P_ * Ft + Q_; //lesson 5, section 9
}

void KalmanFilter::Update(const VectorXd &z) {
		VectorXd y = z - H_ * x_;
		MatrixXd Ht = H_.transpose();
		MatrixXd S = H_ * P_ * Ht + R_;
		MatrixXd Si = S.inverse();
		MatrixXd K =  P_ * Ht * Si;  

    MatrixXd I;
    int xsize = x_.size();
    I = MatrixXd::Identity(xsize, xsize);


		//new state
		x_ = x_ + (K * y);
		P_ = (I - K * H_) * P_;    
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt((x * x) + (y * y));

  if(rho < 0.0001){
    rho = 0.0001;
  }

  float theta = atan2(y, x);
  float ro_dot = ((x * vx) + (y *vy)) / rho; 
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;

  VectorXd y_vector = z - z_pred;

  y_vector[1] = atan2(sin(y_vector[1]),cos(y_vector[1]));

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;  

  MatrixXd I;
  int xsize = x_.size();
  I = MatrixXd::Identity(xsize, xsize);
  //new state
  x_ = x_ + (K * y_vector);
  P_ = (I - K * H_) * P_;  
}
