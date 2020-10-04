#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ekf.h"
#include "input_data.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name) 
{
  if (!in_file.is_open()) 
  {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }
  if (!out_file.is_open()) 
  {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main()
{
  // Check if we can open files
  string in_file_name_ = "/home/gowithrobo/0_ups/projects/cpp_sensorfusion/data/drone_dataset.csv";
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);
  string out_file_name_ = "output.txt";
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);
  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<InputData> input_data_list;
  string line;
  while (getline(in_file_, line)) 
  {
    InputData pred_input;
    istringstream iss(line);
    float timestamp;
    iss >> timestamp;
    pred_input.timestamp_ = timestamp;
    
    pred_input.imu_ = VectorXd(6); 
    float ang_vel_x;
    float ang_vel_y;
    float ang_vel_z;
    float lin_vel_x;
    float lin_vel_y;
    float lin_vel_z;
    iss >> ang_vel_x;
    iss >> ang_vel_y;
    iss >> ang_vel_z;
    iss >> lin_vel_x;
    iss >> lin_vel_y;
    iss >> lin_vel_z;
    pred_input.vicon_ = VectorXd(6); 
    float w_pos_x;
    float w_pos_y;
    float w_pos_z;
    float w_ori_x;
    float w_ori_y;
    float w_ori_z;
    iss >> w_pos_x;
    iss >> w_pos_y;
    iss >> w_pos_z;
    iss >> w_ori_x;
    iss >> w_ori_y;
    iss >> w_ori_z;
    pred_input.imu_ << ang_vel_x, ang_vel_y, ang_vel_z, lin_vel_x, lin_vel_y, lin_vel_z; 
    pred_input.vicon_ << w_pos_x, w_pos_y, w_pos_z, w_ori_x, w_ori_y, w_ori_z;
    input_data_list.push_back(pred_input);
  }

  // Create EKF instance
  EKF ekf;
  ekf.predict(input_data_list[0]);

  // //Call the EKF-based fusion
  size_t N = input_data_list.size();
  for (size_t k = 0; k < N; ++k) {
    ekf.predict(input_data_list[k]);
    ekf.update(input_data_list[k]);
    ekf.x_ = ekf.x_update_;
    ekf.P_ = ekf.P_update_;
    // output the estimation
    out_file_ << ekf.x_(0) << "\t";
    out_file_ << ekf.x_(1) << "\t";
    out_file_ << ekf.x_(2) << "\t";
    out_file_ << ekf.x_(3) << "\t";
    out_file_ << ekf.x_(4) << "\t";
    out_file_ << ekf.x_(5) << "\t";
    out_file_ << ekf.x_(6) << "\t";
    out_file_ << ekf.x_(7) << "\t";
    out_file_ << ekf.x_(8) << "\t";
    out_file_ << ekf.x_(9) << "\t";
    out_file_ << ekf.x_(10) << "\t";
    out_file_ << ekf.x_(11) << "\t";
    out_file_ << ekf.x_(12) << "\t";
    out_file_ << ekf.x_(13) << "\t";
    out_file_ << ekf.x_(14) << "\n";
  }

  // close files
  if (out_file_.is_open()) 
  {
    out_file_.close();
  }

  if (in_file_.is_open()) 
  {
    in_file_.close();
  }

  return 0;
}
