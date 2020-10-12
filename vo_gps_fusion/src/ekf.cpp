#include "ekf.h"
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
EKF::EKF() 
{
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Covariance matrix initialization.
  P_= MatrixXd(6,6); 
  Q_ = MatrixXd(6,6);
  R_ = MatrixXd(3, 3);
  P_ << 1,0,0,0,0,0,         
        0,1,0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;
        
  Q_ << 1e-3,0,0,0,0,0,
        0,1e-3,0,0,0,0,
        0,0,1e-3,0,0,0,
        0,0,0,1e-3,0,0,
        0,0,0,0,1e-3,0,
        0,0,0,0,0,1e-3;
       
  R_ << 1e-3,0,0,
        0,1e-3,0,
        0,0,1e-3;
}

/**
* Destructor.
*/
EKF::~EKF() {}

void EKF::predict(const InputData& input_data)
{
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) 
  {
    cout << "EKF: " << endl;
    x_ = VectorXd(6);
    x_ << 0,0,0,0,0,0;
    x_(0) = input_data.gpsins_(0);  // pos_x
    x_(1) = input_data.gpsins_(1);  // pos_y
    x_(2) = input_data.gpsins_(2);  // ori_z
    previous_timestamp_ = input_data.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //compute the time elapsed between the current and previous
  dt = (input_data.timestamp_ - previous_timestamp_);
  previous_timestamp_ = input_data.timestamp_;
  xdot_ = VectorXd(6);
  xdot_ << 0,0,0,0,0,0;
  xdot_(0) = input_data.vo_(0);
  xdot_(1) = input_data.vo_(1);
  xdot_(2) = input_data.vo_(2);

  Tools tools;
  MatrixXd A_cal;
  MatrixXd U_cal;
  tools.CalculateJacobian(A_cal, U_cal);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  MatrixXd F = I + dt * A_cal;
  MatrixXd F_transpose = F.transpose();
  MatrixXd U_cal_transpose = U_cal.transpose();
  // Prediction step output: mean(x_prediction), covariance(P_prediction)
  x_prediction_ = x_ + dt*xdot_;
  P_prediction_ = F*P_*F_transpose + U_cal * dt * Q_ * U_cal_transpose;
}

void EKF::update(const InputData& input_data)
{
  C_ = MatrixXd(3, 6);
  C_ << 1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0;

  MatrixXd C_transpose = C_.transpose();
  MatrixXd K = P_prediction_*C_transpose* ((C_*P_prediction_*C_transpose + R_).inverse());
  x_update_ = x_prediction_ + K*(input_data.gpsins_ - C_*x_prediction_);
  P_update_ = P_prediction_ - (K*C_*P_prediction_);
            
}
