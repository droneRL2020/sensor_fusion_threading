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
  string out_file_name_ = "output.csv";
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
    
    pred_input.vo_ = VectorXd(3); 
    float lin_vel_x;
    float lin_vel_y;
    float ang_vel_z;
    iss >> lin_vel_x;
    iss >> lin_vel_y;
    iss >> ang_vel_z;
    pred_input.gpsins_ = VectorXd(3); 
    float pos_x;
    float pos_y;
    float ori_z;
    iss >> pos_x;
    iss >> pos_y;
    iss >> ori_z;
    pred_input.vo_ << lin_vel_x, lin_vel_y, ang_vel_z; 
    pred_input.gpsins_ << pos_x, pos_y, ori_z;
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
    out_file_ << ekf.x_(5) << "\n";
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
