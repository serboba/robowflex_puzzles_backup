//
// Created by serboba on 28.01.22.
//

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Dense>
#include <vector>
//#include <python2.7/Python.h>
#include <Python.h>
#ifndef ROBOWFLEX_DART_URDF_READ_H
#define ROBOWFLEX_DART_URDF_READ_H



class URDF_IO{

    public:
        std::vector<std::string> group_names;
        std::vector<std::vector<int>> group_indices;
        std::vector<double> goal_pose;

        URDF_IO(std::string filename);
};




#endif //ROBOWFLEX_DART_URDF_READ_H
