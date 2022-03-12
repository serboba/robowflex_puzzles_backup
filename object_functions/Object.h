//
// Created by serboba on 12.11.21.
//

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <random>

#include <Eigen/Dense>
#include <robowflex_library/util.h>
#include <robowflex_dart/point_collector.h>
#include <tf2/LinearMath/Quaternion.h>


#ifndef ROBOWFLEX_DART_OBJECT_H
#define ROBOWFLEX_DART_OBJECT_H

using namespace Eigen;

struct Link{
    std::string name;
    Vector3d size,pos,rpy;

    Link(std::string name_, Vector3d size_,Vector3d pos_,Vector3d rpy_) : name(name_),size(size_),pos(pos_),rpy(rpy_) {}
};

struct Joint{
    std::string type,name;
    Vector3d pos,rpy;
    Vector3d axis;

    Joint(std::string type_,std::string name_,Vector3d pos_,Vector3d rpy_, Vector3d axis_) : name(name_), type(type_),pos(pos_),rpy(rpy_),axis(axis_) {}
};


class Object {
    public:

        //tf2::Quaternion actual_rotation;

        Joint joint;
        Link link;
        std::string group_name;

        Vector3d actual_position,actual_rotation;

        Object(std::string l_name,Eigen::Vector3d l_size,Eigen::Vector3d l_xyz, Eigen::Vector3d l_rpy,
               std::string j_name, Eigen::Vector3d j_xyz, Eigen::Vector3d j_rpy, std::string gr_name,
               std::string j_type, Vector3d j_axis)
               :
               link(l_name,l_size,l_xyz,l_rpy), joint(j_type,j_name,j_xyz,j_rpy,j_axis)
               {
            
            group_name = gr_name;
            actual_position = j_xyz+l_xyz;
            actual_rotation = j_rpy + l_rpy;
            
            std::cout << "ACTUAL POS " << actual_position << std::endl;
            std::cout << "ACTUAL ROT " << actual_rotation << std::endl;
            }

        void get_objects_from_urdf();
};


#endif //ROBOWFLEX_DART_OBJECT_H

