#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class Tools 
{
public:
  /**
  * Constructor.
  */
  Tools();
  /**
  * Destructor.
  */
  virtual ~Tools();
  /**
  * A helper method to calculate Jacobians.
  */
  //Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& f, const Eigen::VectorXd& v, Eigen::MatrixXd & a, Eigen::MatrixXd & u);
  void CalculateJacobian(Eigen::MatrixXd& A_, Eigen::MatrixXd& U_);
  /**
  * A helper method to calculate RMSE.
  */
  // Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
 private:
  // Linearization:jacobian(state_dot, state) --> (state_dim x state_dim)
  Eigen::MatrixXd A_;
  // Linearization:jacobian(state_dot, noise) --> (state_dim x noise_dim)
  Eigen::MatrixXd U_;
};

#endif /* TOOLS_H_ */
