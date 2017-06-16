#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;		// state/position
  P_ = P_in;		// state/position noise/covariance
  F_ = F_in;		// state transition function, Process model matrix
  H_ = H_in;		// Measurement function
  R_ = R_in;		// process model noise/variance
  Q_ = Q_in;   		// sensor measurement noise/covariance
}	

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
	for (unsigned int n = 0; n < measurements.size(); ++n) {

		VectorXd z = measurements[n];
  */
  x_ = F_ * x_;	// + u;		prior
  MatrixXd Ft_ = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;	//state variance

    //std::cout << "x=" << std::endl <<  x << std::endl;
    //std::cout << "P=" << std::endl <<  P << std::endl;
}  

void KalmanFilter::Update(const VectorXd &z) {		//z is sensor measurement
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;			// residual
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;		// system uncertainty
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;			// Kalman gain

  //new state
  x_ = x_ + K * y;				// posterior
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;			// posterior variance
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
//	MatrixXd Hj(3,4); //ncy
//	//recover state parameters
//	float px = ekf_.x_(0);
//	float py = ekf_.x_(1);
//	float vx = ekf_.x_(2);
//	float vy = ekf_.x_(3);

	//pre-compute a set of terms to avoid repeated calculation
	float rho = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
	float phi = atan2(x_(1),x_(0));
	float rho_dot;

//check division by zero
	if (fabs(rho) < 0.0001) {
	  rho_dot = 0;		//
	} else {		//
   rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  if(y(1) > 3.14){
    y(1) = y(1) - 2*3.14;
  }
  else if(y(1) < -3.14){
    y(1) = y(1) + 2*3.14;
  }	


//  VectorXd y = z - H_ * x_;			//ncy
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;		// system uncertainty
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;			// Kalman gain

  //new state
  x_ = x_ + (K * y);				// posterior
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;			// posterior variance
}
