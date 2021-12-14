//
// Created by serboba on 12.12.21.
//

#ifndef ROBOWFLEX_DART_SOLUTION_PARSER_H
#define ROBOWFLEX_DART_SOLUTION_PARSER_H


#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <Eigen/Dense>
#include <robowflex_dart/conversion_functions.h>

#include <robowflex_dart/world.h>


using namespace Eigen;

void main_test();

std::vector<std::pair<std::string,std::vector<double>>> get_domain_values(std::vector<std::string> joint_names);

#endif //ROBOWFLEX_DART_SOLUTION_PARSER_H
