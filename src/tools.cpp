#include <iostream>
#include "tools.h"
#include <math.h>       /* atan2 */

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


void Tools::CalculateJacobian(const VectorXd& f, const VectorXd& v, MatrixXd& A_, MatrixXd& U_)
{
  float x = v(0);
  float y = v(1);
  float z = v(2);
  float q_x = v(3);
  float q_y = v(4);
  float q_z = v(5);
  float v_x = v(6);
  float v_y = v(7);
  float v_z = v(8);
  float bg_x = v(9);
  float bg_y = v(10);
  float bg_z = v(11);
  float ba_x = v(12);
  float ba_y = v(13);
  float ba_z = v(14);
  float w_x = f(3);
  float w_y = f(4);
  float w_z = f(5);
  float a_x = f(6);
  float a_y = f(7);
  float a_z = f(8);
  float ng_x = 0;
  float ng_y = 0;
  float ng_z = 0;
  float na_x = 0;
  float na_y = 0;
  float na_z = 0;
  A_ = MatrixXd(15,15);
  A_ << 0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 1, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 1, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 1,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                         (sin(q_y)*(bg_x + ng_x - w_x))/(pow(cos(q_y),2) + pow(sin(q_y),2)) - (cos(q_y)*(bg_z + ng_z - w_z))/(pow(cos(q_y),2) + pow(sin(q_y),2)),                                                                                                                                                                                   0, 0, 0, 0,                              -cos(q_y)/(pow(cos(q_y),2) + pow(sin(q_y),2)),  0,                             -sin(q_y)/(pow(cos(q_y),2) + pow(sin(q_y),2)),                                                0,                  0,                                                0,
        0, 0, 0, (cos(q_x)*cos(q_y)*(bg_z + ng_z - w_z))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)) - (cos(q_x)*sin(q_y)*(bg_x + ng_x - w_x))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)) + (cos(q_y)*sin(q_x)*(sin(q_x)*pow(cos(q_y),2) + sin(q_x)*pow(sin(q_y),2))*(bg_z + ng_z - w_z))/pow((cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),2) - (sin(q_x)*sin(q_y)*(sin(q_x)*pow(cos(q_y),2) + sin(q_x)*pow(sin(q_y),2))*(bg_x + ng_x - w_x))/pow((cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),2), - (cos(q_y)*sin(q_x)*(bg_x + ng_x - w_x))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)) - (sin(q_x)*sin(q_y)*(bg_z + ng_z - w_z))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),                                                                                                                                                                                   0, 0, 0, 0, -(sin(q_x)*sin(q_y))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)), -1, (cos(q_y)*sin(q_x))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                               (sin(q_y)*(sin(q_x)*pow(cos(q_y),2) + sin(q_x)*pow(sin(q_y),2))*(bg_x + ng_x - w_x))/pow((cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),2) - (cos(q_y)*(sin(q_x)*pow(cos(q_y),2) + sin(q_x)*pow(sin(q_y),2))*(bg_z + ng_z - w_z))/pow((cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),2),                     (cos(q_y)*(bg_x + ng_x - w_x))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)) + (sin(q_y)*(bg_z + ng_z - w_z))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),                                                                                                                                                                                   0, 0, 0, 0,             sin(q_y)/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),  0,           -cos(q_y)/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                           cos(q_x)*sin(q_y)*sin(q_z)*(ba_x - a_x + na_x) - cos(q_x)*cos(q_y)*sin(q_z)*(ba_z - a_z + na_z) - cos(q_z)*sin(q_x)*(ba_y - a_y + na_y),                                 (cos(q_z)*sin(q_y) + cos(q_y)*sin(q_x)*sin(q_z))*(ba_x - a_x + na_x) - (cos(q_y)*cos(q_z) - sin(q_x)*sin(q_y)*sin(q_z))*(ba_z - a_z + na_z), (cos(q_y)*sin(q_z) + cos(q_z)*sin(q_x)*sin(q_y))*(ba_x - a_x + na_x) + (sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z)*sin(q_x))*(ba_z - a_z + na_z) - cos(q_x)*sin(q_z)*(ba_y - a_y + na_y), 0, 0, 0,                                                                0,  0,                                                               0,   sin(q_x)*sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z),  cos(q_x)*cos(q_z), - cos(q_z)*sin(q_y) - cos(q_y)*sin(q_x)*sin(q_z),
        0, 0, 0,                                                                                                                                                                                                                                                                                                           cos(q_z)*sin(q_x)*(ba_y - a_y + na_y) + cos(q_x)*cos(q_y)*cos(q_z)*(ba_z - a_z + na_z) - cos(q_x)*cos(q_z)*sin(q_y)*(ba_x - a_x + na_x),                                 (sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z)*sin(q_x))*(ba_x - a_x + na_x) - (cos(q_y)*sin(q_z) + cos(q_z)*sin(q_x)*sin(q_y))*(ba_z - a_z + na_z), cos(q_x)*sin(q_z)*(ba_y - a_y + na_y) - (cos(q_z)*sin(q_y) + cos(q_y)*sin(q_x)*sin(q_z))*(ba_z - a_z + na_z) - (cos(q_y)*cos(q_z) - sin(q_x)*sin(q_y)*sin(q_z))*(ba_x - a_x + na_x), 0, 0, 0,                                                                0,  0,                                                               0, - cos(q_y)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y), -cos(q_x)*cos(q_z),   cos(q_y)*cos(q_z)*sin(q_x) - sin(q_y)*sin(q_z),
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                     cos(q_y)*sin(q_x)*(ba_z - a_z + na_z) - cos(q_x)*(ba_y - a_y + na_y) - sin(q_x)*sin(q_y)*(ba_x - a_x + na_x),                                                                                               cos(q_x)*sin(q_y)*(ba_z - a_z + na_z) + cos(q_x)*cos(q_y)*(ba_x - a_x + na_x),                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                cos(q_x)*sin(q_y),          -sin(q_x),                               -cos(q_x)*cos(q_y),
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0,
        0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                 0,                                                                                                                                                                           0,                                                                                                                                                                                   0, 0, 0, 0,                                                                0,  0,                                                               0,                                                0,                  0,                                                0;
  U_ = MatrixXd(15, 12);
  U_ <<                                                              0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 0,
                                                                    0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 0,
                                                                    0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 0,
                                 -cos(q_y)/(pow(cos(q_y),2) + pow(sin(q_y),2)),  0,                             -sin(q_y)/(pow(cos(q_y),2) + pow(sin(q_y),2)),                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 0,
  -(sin(q_x)*sin(q_y))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)), -1, (cos(q_y)*sin(q_x))/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 0,
             sin(q_y)/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),  0,           -cos(q_y)/(cos(q_x)*pow(cos(q_y),2) + cos(q_x)*pow(sin(q_y),2)),                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 0,
                                                                0,  0,                                                               0,   sin(q_x)*sin(q_y)*sin(q_z) - cos(q_y)*cos(q_z),  cos(q_x)*cos(q_z), - cos(q_z)*sin(q_y) - cos(q_y)*sin(q_x)*sin(q_z), 0, 0, 0, 0, 0, 0,
                                                                0,  0,                                                               0, - cos(q_y)*sin(q_z) - cos(q_z)*sin(q_x)*sin(q_y), -cos(q_x)*cos(q_z),   cos(q_y)*cos(q_z)*sin(q_x) - sin(q_y)*sin(q_z), 0, 0, 0, 0, 0, 0,
                                                                0,  0,                                                               0,                                cos(q_x)*sin(q_y),          -sin(q_x),                               -cos(q_x)*cos(q_y), 0, 0, 0, 0, 0, 0,
                                                                0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 0, 1, 0, 0,
                                                                0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 0, 0, 1, 0,
                                                                0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 0, 0, 0, 1,
                                                                0,  0,                                                               0,                                                0,                  0,                                                0, 1, 0, 0, 0, 0, 0,
                                                                0,  0,                                                               0,                                                0,                  0,                                                0, 0, 1, 0, 0, 0, 0,
                                                                0,  0,                                                               0,                                                0,                  0,                                                0, 0, 0, 1, 0, 0, 0;
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