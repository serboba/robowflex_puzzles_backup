//
// Created by serboba on 03.11.21.
//

#ifndef ROBOWFLEX_DART_POINT_COLLECTOR_H
#define ROBOWFLEX_DART_POINT_COLLECTOR_H

#include <robowflex_dart/Object.h>
#include <Eigen/Dense>

using namespace Eigen;

//Eigen::MatrixXd get_random_pose(int object_no);

Eigen::MatrixXd get_pose_object (Object &obj, int surface_no);

MatrixXd get_rotated_vertex(Vector3d obj_rpy, Vector3d point, Vector3d joint_pos );

std::vector<double> read_into_vector(std::string str);

void create_txt_from_urdf();

#endif //ROBOWFLEX_DART_POINT_COLLECTOR_H

