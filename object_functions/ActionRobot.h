//
// Created by serboba on 11.03.22.
//

#ifndef ROBOWFLEX_DART_ACTIONROBOT_H
#define ROBOWFLEX_DART_ACTIONROBOT_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <random>

#include <Eigen/Dense>
#include <robowflex_dart/point_collector.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace Eigen;


class ActionRobot {

public:

    Vector3d move;
    MatrixXd rotate;
    std::string obj_name;
    std::string obj_group_name;

        ActionRobot(Vector3d movement_, MatrixXd rotation_, std::string obj_name_, std::string obj_group_name) : move(movement_),
        rotate(rotation_),obj_name(obj_name_),obj_group_name(obj_group_name)
        {

        }

        ActionRobot(MatrixXd rotation_, std::string obj_name_, std::string obj_group_name) :
        rotate(rotation_),obj_name(obj_name_),obj_group_name(obj_group_name)
        {
            move = Vector3d(0.0,0.0,0.0);
        }


        void translateActions();

};

int indexToGroup(std::vector<double> s_from, std::vector<double> s_to, std::vector<std::vector<int>> group_indices);

void getActionsFromPath(std::string filename, std::vector<std::vector<int>> group_indices);



#endif //ROBOWFLEX_DART_ACTIONROBOT_H

