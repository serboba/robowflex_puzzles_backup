//
// Created by serboba on 03.11.21.
//

#ifndef ROBOWFLEX_DART_POINT_COLLECTOR_H
#define ROBOWFLEX_DART_POINT_COLLECTOR_H

#include <robowflex_dart/Object.h>
#include <Eigen/Dense>

using namespace Eigen;

std::vector<Eigen::MatrixXd> get_pose_matrix(Vector3d joint_pos,Vector3d joint_rpy,Vector3d object_xyz,Vector3d object_rpy, Vector3d object_size);

//Eigen::MatrixXd get_random_pose(int object_no);
std::vector<std::pair<std::string, Eigen::MatrixXd>> get_pose_object (int object_no);
void create_objects_from_urdf();
std::vector<Object> get_objects();

#endif //ROBOWFLEX_DART_POINT_COLLECTOR_H
