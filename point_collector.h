//
// Created by serboba on 03.11.21.
//

#ifndef ROBOWFLEX_DART_POINT_COLLECTOR_H
#define ROBOWFLEX_DART_POINT_COLLECTOR_H
#include <Eigen/Dense>
using namespace Eigen;

std::vector<Eigen::MatrixXd> get_pose_matrix(Vector3d joint_pos,Vector3d joint_rpy, Vector3d object_size);

Eigen::MatrixXd get_random_pose(int object_no);
std::vector<std::pair<std::string, Eigen::MatrixXd>> get_pose_object (int object_no);

#endif //ROBOWFLEX_DART_POINT_COLLECTOR_H
