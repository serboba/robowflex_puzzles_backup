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

struct OLink{
    std::string name;
    Vector3d size,pos,rpy;

};

enum JType {revolute , prismatic};


struct OJoint{
    std::string name;
    JType type;
    int direction;
};


struct OJoints{
    Vector3d pos,rpy;
    std::vector<OJoint> joints;
};


class Object {
    public:

        OJoints joints;
        OLink link;
        std::string group_name;

        Vector3d actual_position,actual_rotation;

        Object(std::string gr_name, OLink link_, OJoints joints_) : link(link_), joints(joints_),group_name(gr_name)
               {

            actual_position = link.pos + joints.pos;
            actual_rotation = link.rpy + joints.rpy;

            }

        void get_objects_from_urdf();
};


#endif //ROBOWFLEX_DART_OBJECT_H

