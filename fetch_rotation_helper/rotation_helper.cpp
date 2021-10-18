//
// Created by serboba on 16.10.21.
//


#include <robowflex_dart/tsr.h>
#include <robowflex_dart/rotation_helper.h>
#include "math.h"


static const double deg_90 = 1.5708;
static const double n_deg_90 = -1.5708;

static const double deg_180 = 3.14159;
static const double n_deg_180 = -3.14159;

using namespace robowflex;


Eigen::Quaterniond Rotation_Helper::calcQuaternion(std::vector<double> vec){
    Eigen::Quaterniond newRotation;
    newRotation.w() = round((vec[0]*vec[1]*vec[2] - vec[3]*vec[4]*vec[5])*10000.0 ) / 10000.0;
    newRotation.x() = round((vec[3]*vec[4]*vec[2] + vec[0]*vec[1]*vec[5])*10000.0 ) / 10000.0;
    newRotation.y() = round((vec[3]*vec[1]*vec[2] + vec[0]*vec[4]*vec[5])*10000.0 ) / 10000.0;
    newRotation.z() = round((vec[0]*vec[4]*vec[2] - vec[3]*vec[1]*vec[5])*10000.0 ) / 10000.0;

    return newRotation;
}

Eigen::Quaterniond Rotation_Helper::calculateCS(std::vector<double> rotation_deg){
    std::vector<double> cs_vec(6);

    cs_vec[0] = cos(rotation_deg[0]/2);
    cs_vec[1] = cos(rotation_deg[1]/2);
    cs_vec[2] = cos(rotation_deg[2]/2);
    cs_vec[3] = sin(rotation_deg[0]/2);
    cs_vec[4] = sin(rotation_deg[1]/2);
    cs_vec[5] = sin(rotation_deg[2]/2);

    return calcQuaternion(cs_vec);
}

std::vector<double> Rotation_Helper::getDegrees(Eigen::Quaterniond q){
    std::vector<double> rot(3);

    if((q.x()*q.y() +q.z()*q.w()) == 0.5){
        rot[0] = 2*atan2(q.x(),q.w());
        rot[1] = asin(2*q.x()*q.y() + 2*q.z()*q.w());
        rot[2] = 0;
    }else if((q.x()*q.y() +q.z()*q.w()) == -0.5){
        rot[0] = -2*atan2(q.x(),q.w());
        rot[1] = asin(2*q.x()*q.y() + 2*q.z()*q.w());
        rot[2] = 0;
    }else{

        rot[0] = atan2(2*q.y()*q.w() - 2*q.x()*q.z() , 1 - 2*pow(q.y(),2) - 2*pow(q.z(),2));
        rot[1] = asin(2*q.x()*q.y() + 2*q.z()*q.w());
        rot[2] = atan2(2*q.x()*q.w() - 2*q.y()*q.z(), 1- 2* pow(q.x(),2)- 2* pow(q.z(),2));
   }

    return rot;
}
// 90 DEGREES 1.5715

Eigen::Quaterniond Rotation_Helper::rotateRight(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);  // heading,attitude,bank of old rotation
    if(newRotation[2] == 2*deg_180)
        newRotation[2] = 0.0;
    newRotation[2] += deg_90; //bank +90 rotateright
    Rotation_Helper::roundValues(newRotation);

    return Rotation_Helper::calculateCS(newRotation);
}

Eigen::Quaterniond Rotation_Helper::rotateLeft(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);  // heading,attitude,bank of old rotation
    if(newRotation[2] == 2*n_deg_180)
        newRotation[2] = 0.0;
    newRotation[2] -= deg_90; //bank +90 rotateright
    Rotation_Helper::roundValues(newRotation);
    return Rotation_Helper::calculateCS(newRotation);
}

void Rotation_Helper::roundValues(std::vector<double> &values){
    for(int i =0 ; i<3;i++){
        round( values[i] * 10000.0 ) / 10000.0;
    }
}

/*

bool getDirection(std::vector<double> rotation){

    //TODO ROTATE WHILE LOOKING STRAIGHT rot[0] = 0
    //TODO ROTATE WHILE LOOKING UP       rot[0] = -1.57 ?
    // DONE ROTATE WHILE LOOKING DOWN (heading 90) rot[0] 1.57
}

Eigen::Quaterniond rotateUp(){
    // TODO
}

Eigen::Quaterniond rotateDown(){
  // TODO
}

*/