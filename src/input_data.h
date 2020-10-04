#ifndef INPUT_DATA_H_
#define INPUT_DATA_H_

#include "Eigen/Dense"

class InputData {
public:
  float timestamp_;
  Eigen::VectorXd imu_;
  //Eigen::VectorXd all_;  // state, ang_vel, lin_acc, noise
  Eigen::VectorXd vicon_;
};
#endif /* INPUT_DATA_H_ */
