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

struct ActionR{ // action for robot

    Vector3d pos,rpy;
    std::string target_link_name;
    int obj_index;

    ActionR(Vector3d pos,Vector3d rpy, std::string link_, int index_) : pos(pos), rpy(rpy), target_link_name(link_), obj_index(index_){}
};

struct ActionP{ // action from path
    int group_index;
    std::vector<double> vals;

    ActionP(int index_, std::vector<double> vals_) : group_index(index_), vals(vals_){}
};

class ActionRobot {

public:

    ActionR action;

        ActionRobot(ActionR action_) : action(action_){}


        void translateActions();

};

int indexToGroup(std::vector<double> s_from, std::vector<double> s_to, std::vector<std::vector<int>> group_indices);

void getActionsFromPath(std::string filename, std::vector<std::vector<int>> group_indices,std::vector<ActionP> &actions_);


void translateActions(std::vector<ActionP> &actions_, std::vector<Object> objects_, std::vector<ActionR> &actions_robot);

#endif //ROBOWFLEX_DART_ACTIONROBOT_H

