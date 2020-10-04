#ifndef EKF_H_
#define EKF_H_

#include "input_data.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

class EKF {
public:
  // state vector                             --> (state_dim x 1)
  Eigen::VectorXd x_;
  // state_dot vector                         --> (state_dim x 1)
  Eigen::VectorXd xdot_;
  // noise vector                             --> (noise_dim x 1)
  // Eigen::VectorXd noise_;
  // state covariance matrix                  --> (state_dim x state_dim)
  Eigen::MatrixXd P_;
  // process noise covariance matrix          --> (state_dim x state_dim)
  Eigen::MatrixXd Q_;
  // measurement noise covariance matrix      --> (measurement_dim x measurement_dim)
  Eigen::MatrixXd R_;
  // // Discretization: I + dt * A_cal           --> (state_dim x state_dim)
  // Eigen::MatrixXd F_;
  // // Linearization:jacobian(state_dot, state) --> (state_dim x state_dim)
  // Eigen::MatrixXd A_;
  // // Linearization:jacobian(state_dot, noise) --> (state_dim x noise_dim)
  // Eigen::MatrixXd U_;
  // Linearization:measurement matrix         --> (measurement_dim x state_dim)
  Eigen::MatrixXd C_;
  float dt;
  /**
  * Constructor.
  */
  EKF();
  /**
  * Destructor.
  */
  virtual ~EKF();
  Eigen::VectorXd x_update_;
  Eigen::MatrixXd P_update_;
  void predict(const InputData& input_data);
  void update(const InputData& input_data);

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;
  // previous timestamp
  float previous_timestamp_;
  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::VectorXd x_prediction_;
  Eigen::MatrixXd P_prediction_;  
};

#endif /* EKF_H_ */
