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

Eigen::MatrixXd get_pose_object (Object &obj);

void create_objects_from_urdf();
std::vector<Object> get_objects();
void set_objects(std::vector<Object> objects_);

MatrixXd get_rotated_vertex(Vector3d obj_rpy, Vector3d point, Vector3d joint_pos );
Quaterniond get_quaternion_from_euler(float yaw, float pitch, float roll);

MatrixXd new_rotation_quaternion(MatrixXd rpy, Quaterniond q_original, int surface_no);
    void create_txt_from_urdf();

#endif //ROBOWFLEX_DART_POINT_COLLECTOR_H
