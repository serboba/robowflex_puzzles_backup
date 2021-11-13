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


#ifndef ROBOWFLEX_DART_OBJECT_H
#define ROBOWFLEX_DART_OBJECT_H


class Object {
    public:

        std::string link_name;
        Eigen::Vector3d link_size,link_xyz,link_rpy;
        std::string joint_name, group_name;
        Eigen::Vector3d joint_xyz,joint_rpy;

        Object(std::string l_name,Eigen::Vector3d l_size,Eigen::Vector3d l_xyz, Eigen::Vector3d l_rpy,
               std::string j_name, Eigen::Vector3d j_xyz, Eigen::Vector3d j_rpy, std::string gr_name){
            link_name = l_name;
            link_size = l_size;
            link_xyz = l_xyz;
            link_rpy = l_rpy;

            joint_name = j_name;
            joint_xyz = j_xyz;
            joint_rpy = j_rpy;
            group_name = gr_name;
            }

        void get_objects_from_urdf();
};


#endif //ROBOWFLEX_DART_OBJECT_H
