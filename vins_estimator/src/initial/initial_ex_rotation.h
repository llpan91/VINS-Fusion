/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <vector>
#include "../estimator/parameters.h"
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <ros/console.h>

/* This class help you to calibrate extrinsic rotation between imu and camera when your totally 
 * don't konw the extrinsic parameter */
class InitialEXRotation {
 public:
  InitialEXRotation();
  
  
  /// \param ric
  bool CalibrationExRotation(const std::vector<pair<Vector3d, Vector3d>> corres, Quaterniond delta_q_imu,
                             Matrix3d &calib_ric_result);

 private:
  /// \brief solve Relative Rotation 
  Matrix3d solveRelativeR(const vector<pair<Vector3d, Vector3d>> &corres);

  /// \brief return ratio of valid pt(in both front of two frame) / all pts 
  double testTriangulation(const vector<cv::Point2f> &l, const vector<cv::Point2f> &r,
			   cv::Mat_<double> R, cv::Mat_<double> t);
  void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2, cv::Mat_<double> &t1,
                  cv::Mat_<double> &t2);
  int frame_count;

  /// \brief Rotation from frame(k) to frame(k+1)
  
  // delta_q_cam_computational
  vector<Matrix3d> Rc;
  // delta_q_imu
  vector<Matrix3d> Rimu;
  // delta_q_cam_converted
  vector<Matrix3d> Rc_g;
  
  // rotation from imu(body) to camera
  Matrix3d ric;
};
