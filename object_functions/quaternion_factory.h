//
// Created by serboba on 21.11.21.
//

#ifndef ROBOWFLEX_DART_QUATERNION_FACTORY_H
#define ROBOWFLEX_DART_QUATERNION_FACTORY_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;

MatrixXd rpy_to_quaternion(double roll, double pitch, double yaw);
MatrixXd quaternion_x_y_z(MatrixXd rpy);
MatrixXd quaternion_y_z(MatrixXd rpy);
MatrixXd quaternion_z(MatrixXd rpy);
MatrixXd get_quaternion_unten(MatrixXd rpy);
MatrixXd get_quaternion_links(MatrixXd rpy);
MatrixXd get_quaternion_hinten(MatrixXd rpy);
MatrixXd get_quaternion_rechts(MatrixXd rpy);
MatrixXd get_quaternion_vorne(MatrixXd rpy);
MatrixXd get_quaternion_oben(MatrixXd rpy);

#endif //ROBOWFLEX_DART_QUATERNION_FACTORY_H

