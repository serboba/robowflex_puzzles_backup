//
// Created by serboba on 31.12.21.
//

#ifndef ROBOWFLEX_DART_ISOSPACE_H
#define ROBOWFLEX_DART_ISOSPACE_H


#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/joints.h>


namespace robowflex
{
    namespace darts
    {

       void interpolate_iso(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state);

    }
}


#endif //ROBOWFLEX_DART_ISOSPACE_H
