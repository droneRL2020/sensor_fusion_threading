#include <iostream>
#include "tools.h"
#include <math.h>       /* atan2 */

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


void Tools::CalculateJacobian(MatrixXd& A_, MatrixXd& U_)
{
  A_ = MatrixXd(6,6);
  A_ <<  0, 0, 0, -1,  0,  0,
         0, 0, 0,  0, -1,  0,
         0, 0, 0,  0,  0, -1,
         0, 0, 0,  0,  0,  0,
         0, 0, 0,  0,  0,  0,
         0, 0, 0,  0,  0,  0;

  U_ = MatrixXd(5, 5);
  U_ <<   -1,  0,  0, 0, 0,
          0, -1,  0, 0, 0,
          0,  0, -1, 0, 0,
          0,  0,  0, 1, 0,
          0,  0,  0, 0, 1;
}


// VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
//                               const vector<VectorXd> &ground_truth) {
//   /**
//   TODO:
//     * Calculate the RMSE here.
//   */
//   VectorXd rmse(4);
//   rmse << 0, 0, 0, 0;

//   // check the validity of the following inputs:
//   //  * the estimation vector size should not be zero
//   //  * the estimation vector size should equal ground truth vector size
//   if (estimations.size() != ground_truth.size()
//       || estimations.size() == 0){
//       std::cout << "Invalid estimation or ground_truth data" << std::endl;
//       return rmse;
//   }

//   //accumulate squared residuals
//   for (unsigned int i = 0; i < estimations.size(); ++i){

//       VectorXd residual = estimations[i] - ground_truth[i];

//       //coefficient-wise multiplication
//       residual = residual.array()*residual.array();
//       rmse += residual;
//   }

//   //calculate the mean
//   rmse = rmse / estimations.size();

//   //calculate the squared root
//   rmse = rmse.array().sqrt();

//   //return the result
//   return rmse;
// }