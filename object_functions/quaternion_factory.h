//
// Created by serboba on 21.11.21.
//

#ifndef ROBOWFLEX_DART_QUATERNION_FACTORY_H
#define ROBOWFLEX_DART_QUATERNION_FACTORY_H

#include <stdlib.h>
#include <Eigen/Dense>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <robowflex_dart/conversion_functions.h>
using namespace Eigen;

MatrixXd rpy_to_quaternion(double roll, double pitch, double yaw);

std::vector<MatrixXd> rpy_to_vector(MatrixXd rpy);
MatrixXd quaternion_x_y_z(MatrixXd rpy);
MatrixXd match_deg_to_rpy(MatrixXd rpy, MatrixXd axis);
MatrixXd get_quaternion_unten(MatrixXd rpy);
MatrixXd get_quaternion_links(MatrixXd rpy);
MatrixXd get_quaternion_hinten(MatrixXd rpy);
MatrixXd get_quaternion_rechts(MatrixXd rpy);
MatrixXd get_quaternion_vorne(MatrixXd rpy);
MatrixXd get_quaternion_oben(MatrixXd rpy);

MatrixXd rmv(MatrixXd rpy, MatrixXd quaternion);


MatrixXd actual_quaternion(MatrixXd obj_rpy);
MatrixXd actual_quaternion(MatrixXd obj_rpy, int surf_no);
MatrixXd match_deg_to_rpy_new(MatrixXd rpy, MatrixXd obj_rpy, int surface_no);
#endif //ROBOWFLEX_DART_QUATERNION_FACTORY_H


