//
// Created by serboba on 21.11.21.
//

#ifndef ROBOWFLEX_DART_CONVERSION_FUNCTIONS_H
#define ROBOWFLEX_DART_CONVERSION_FUNCTIONS_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <random>


#include <Eigen/Dense>
#include <robowflex_library/util.h>
#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/quaternion_factory.h>
#include <robowflex_dart/Object.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace Eigen;


void tf_quaternion_to_rpy(tf2::Quaternion q );

void eigen_quaternion_to_rpy(Eigen::Quaterniond q );

void eigen_matrix_quaternion_to_rpy(Eigen::MatrixXd q );

MatrixXd eigen_quaternion_to_matrix(Eigen::Quaterniond quaternion);

tf2::Quaternion eigen_to_tfquaternion(Eigen::MatrixXd quaternion);

tf2::Quaternion eigen_to_tfquaternion(Eigen::Quaterniond quaternion);

Eigen::Quaterniond matrix_to_quaternion(Eigen::MatrixXd matrix);

Eigen::Quaterniond tf_to_eigen_quaternion(tf2::Quaternion quaternion);

Eigen::MatrixXd tf_to_eigen_matrix_q(tf2::Quaternion quaternion);

std::vector<double> eigenvector_to_std(Vector3d vec);

MatrixXd vec_to_matrix(Vector3d vec);

MatrixXd stdvec_to_quat_matrix(std::vector<MatrixXd> quaternion_vec);


std::vector<double> quaternion_matrix_to_stdvec(MatrixXd mat);

Vector3d matrix_to_vec(MatrixXd mat);
#endif //ROBOWFLEX_DART_CONVERSION_FUNCTIONS_H

