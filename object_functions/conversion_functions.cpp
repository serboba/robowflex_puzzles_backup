//
// Created by serboba on 21.11.21.
//

#include <robowflex_dart/conversion_functions.h>


tf2::Quaternion eigen_to_tfquaternion(Eigen::MatrixXd quaternion){
    tf2::Quaternion q_;
    q_.setW(quaternion(0));
    q_.setX(quaternion(1));
    q_.setY(quaternion(2));
    q_.setZ(quaternion(3));
    return q_;
}

MatrixXd eigen_quaternion_to_matrix(Eigen::Quaterniond quaternion){
    MatrixXd q_(1,4);
    q_ << quaternion.w(),quaternion.x(), quaternion.y(),quaternion.z();
    return q_;
}

tf2::Quaternion eigen_to_tfquaternion(Eigen::Quaterniond quaternion){
    tf2::Quaternion q_;
    q_.setW(quaternion.w());
    q_.setX(quaternion.x());
    q_.setY(quaternion.y());
    q_.setZ(quaternion.z());
    return q_;
}

Eigen::Quaterniond matrix_to_quaternion(Eigen::MatrixXd matrix){
    Quaterniond q;
    q.w() = matrix(0);
    q.x() = matrix(1);
    q.y() = matrix(2);
    q.z() = matrix(3);
    return q;
}

void tf_quaternion_to_rpy(tf2::Quaternion q ){
    tf2Scalar roll, pitch, yaw;
    tf2::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw,pitch,roll); // achtung returned Y X Z, bei mir R=x P=y Y=z XYZ deswegen PRY
    std::cout << "ROLL : " << roll << " - PITCH : " << pitch << " - YAW: " << yaw << std::endl;
}

void eigen_quaternion_to_rpy(Eigen::Quaterniond q ){
    tf2Scalar roll, pitch, yaw;
    tf2::Quaternion q_ = eigen_to_tfquaternion(q);
    tf2::Matrix3x3 mat(q_);
    mat.getEulerYPR(yaw,pitch,roll); // achtung returned Y X Z, bei mir R=x P=y Y=z XYZ deswegen PRY
    std::cout << "ROLL : " << roll << " - PITCH : " << pitch << " - YAW: " << yaw << std::endl;
}


void eigen_matrix_quaternion_to_rpy(Eigen::MatrixXd q ){
    tf2Scalar roll, pitch, yaw;
    tf2::Quaternion q_ = eigen_to_tfquaternion(q);
    tf2::Matrix3x3 mat(q_);
    mat.getEulerYPR(yaw,pitch,roll); // achtung returned Y X Z, bei mir R=x P=y Y=z XYZ deswegen PRY
    std::cout << "ROLL : " << roll << " - PITCH : " << pitch << " - YAW: " << yaw << std::endl;
}

Eigen::Quaterniond tf_to_eigen_quaternion(tf2::Quaternion quaternion){
    Eigen::Quaterniond q_;
    q_.w() = quaternion.getW();
    q_.x() = quaternion.getX();
    q_.y() = quaternion.getY();
    q_.z() = quaternion.getZ();
    return q_;
}

Eigen::MatrixXd tf_to_eigen_matrix_q(tf2::Quaternion quaternion){
    Eigen::MatrixXd q_(1,4);
    q_(0) = quaternion.getW();
    q_(1) = quaternion.getX();
    q_(2) = quaternion.getY();
    q_(3) = quaternion.getZ();
    return q_;
}


std::vector<double> eigenvector_to_std(Vector3d vec){
    std::vector<double> degree;
    degree.resize(vec.size());
    VectorXd::Map(&degree[0],vec.size()) = vec;
    return degree;
}

MatrixXd vec_to_matrix(Vector3d vec){
    MatrixXd m(1,3);
    m << vec[0],vec[1],vec[2];
    return m;
}

Vector3d matrix_to_vec(MatrixXd mat){
    Vector3d vec;
    vec << mat(0),mat(1),mat(2);
    return vec;
}

std::vector<double> quaternion_matrix_to_stdvec(MatrixXd mat)
{
    std::vector<double> vec = {mat(0),mat(1),mat(2),mat(3)};
    return vec;
}

MatrixXd stdvec_to_quat_matrix(std::vector<MatrixXd> quaternion_vec){
    MatrixXd mat(6,4);
    mat << quaternion_vec[0], quaternion_vec[1], quaternion_vec[2], quaternion_vec[3], quaternion_vec[4], quaternion_vec[5];
    return mat;
}

