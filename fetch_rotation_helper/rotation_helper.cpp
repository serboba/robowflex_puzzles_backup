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
// rotation_deg = [yaw,pitch,roll]
Eigen::Quaterniond Rotation_Helper::calculateCS(std::vector<double> rotation_deg){
    std::vector<double> cs_vec(6);

    cs_vec[0] = cos(rotation_deg[1]/2);
    cs_vec[1] = cos(rotation_deg[0]/2);
    cs_vec[2] = cos(rotation_deg[2]/2);

    cs_vec[3] = sin(rotation_deg[1]/2);
    cs_vec[4] = sin(rotation_deg[0]/2);
    cs_vec[5] = sin(rotation_deg[2]/2);

    return calcQuaternion(cs_vec);
}
/*
 *  GETDEGREES Q -> (yaw (z-axis) - pitch (y-axis) - roll (x-axis)
 */

std::vector<double> Rotation_Helper::getDegrees(Eigen::Quaterniond q){
    std::vector<double> rot(3);
    // heading 0  , attitude 1 , bank 2
    if((q.x()*q.y() +q.z()*q.w()) == 0.5){
        rot[0] = asin(2*q.x()*q.y() + 2*q.z()*q.w());
        rot[1] = 2*atan2(q.x(),q.w());
        rot[2] = 0;
    }else if((q.x()*q.y() +q.z()*q.w()) == -0.5){
        rot[0] = asin(2*q.x()*q.y() + 2*q.z()*q.w());
        rot[1] = -2*atan2(q.x(),q.w());
        rot[2] = 0;
    }else{
        rot[0] = asin(2*q.x()*q.y() + 2*q.z()*q.w());
        rot[1] = atan2(2*q.y()*q.w() - 2*q.x()*q.z() , 1 - 2*pow(q.y(),2) - 2*pow(q.z(),2));
        rot[2] = atan2(2*q.x()*q.w() - 2*q.y()*q.z(), 1- 2* pow(q.x(),2)- 2* pow(q.z(),2));
  }

    return rot;
}
// 90 DEGREES 1.5715

void Rotation_Helper::roundValues(std::vector<double> &values){
    for(int i =0 ; i<3;i++){
        values[i] = round( values[i] * 10000.0 ) / 10000.0; //omg wie kann ich das vergessen
    }
}


/*
 * Rotate Right? - Left? -> change yaw degree (z-axis) right?(positive) - right?(negative)
 */



Eigen::Quaterniond Rotation_Helper::rotateYawPositive(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);
    Rotation_Helper::roundValues(newRotation);
    if(newRotation[0] == 2*deg_90) // ? TODO CHECK BOUNDS
        return oldRotation; // DO NOTHING
    newRotation[0] += deg_90;
    Rotation_Helper::roundValues(newRotation);
    return Rotation_Helper::calculateCS(newRotation);
}



Eigen::Quaterniond Rotation_Helper::rotateYawNegative(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);
    Rotation_Helper::roundValues(newRotation);
    if(newRotation[0] == 2*n_deg_90) // -90 degrees looking up already, cant rotate up no more
        return oldRotation; // DO NOTHING OUT OF BOUNDS
    newRotation[0] -= deg_90; //heading -90 rotate upwards
    Rotation_Helper::roundValues(newRotation);
    return Rotation_Helper::calculateCS(newRotation);
}



/*
 * Rotate Up - Down -90<=0<=90 -> change pitch degree (y-axis) up(negative) - down(positive)
 */

Eigen::Quaterniond Rotation_Helper::rotateUp(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);
    Rotation_Helper::roundValues(newRotation);
    if(newRotation[1] == n_deg_90) // -90 degrees looking up already, cant rotate up no more
        return oldRotation; // DO NOTHING OUT OF BOUNDS
    newRotation[1] -= deg_90; //heading -90 rotate upwards
    Rotation_Helper::roundValues(newRotation);
    return Rotation_Helper::calculateCS(newRotation);
}

Eigen::Quaterniond Rotation_Helper::rotateDown(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);
    Rotation_Helper::roundValues(newRotation);
    if(newRotation[1] == deg_90) // 90 degrees looking up already, cant rotate down no more
        return oldRotation; // DO NOTHING
    newRotation[1] += deg_90; //heading 90 rotate downwards
    Rotation_Helper::roundValues(newRotation);
    return Rotation_Helper::calculateCS(newRotation);
}


/*
 * Rotate Right - Left -> change roll degree (x-axis) right(positive) - left(negative)
 */

Eigen::Quaterniond Rotation_Helper::rotateRight(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);  // heading,attitude,bank of old rotation
    Rotation_Helper::roundValues(newRotation);
    if(newRotation[2] == 2*deg_180)
        newRotation[2] = 0.0;
    newRotation[2] += deg_90; //bank +90 rotateright
    Rotation_Helper::roundValues(newRotation);

    return Rotation_Helper::calculateCS(newRotation);
}

Eigen::Quaterniond Rotation_Helper::rotateLeft(Eigen::Quaterniond oldRotation){
    std::vector<double> newRotation = Rotation_Helper::getDegrees(oldRotation);  // heading,attitude,bank of old rotation
    Rotation_Helper::roundValues(newRotation);
    if(newRotation[2] == 2*n_deg_180)
        newRotation[2] = 0.0;
    newRotation[2] -= deg_90; //bank +90 rotateright
    Rotation_Helper::roundValues(newRotation);
    return Rotation_Helper::calculateCS(newRotation);
}

