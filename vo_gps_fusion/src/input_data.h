#ifndef INPUT_DATA_H_
#define INPUT_DATA_H_

#include "Eigen/Dense"

class InputData {
public:
  float timestamp_;
  Eigen::VectorXd vo_;
  //Eigen::VectorXd all_;  // state, ang_vel, lin_acc, noise
  Eigen::VectorXd gpsins_;
};
#endif /* INPUT_DATA_H_ */
