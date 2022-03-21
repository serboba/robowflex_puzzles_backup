//
// Created by serboba on 05.01.22.
//

#ifndef ROBOWFLEX_DART_PLANNINGFUNCTIONS_H
#define ROBOWFLEX_DART_PLANNINGFUNCTIONS_H

#include <robowflex_library/builder.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <robowflex_library/class_forward.h>
#include <robowflex_dart/gui.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/ActionRobot.h>
#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/conversion_functions.h>
#include <robowflex_dart/quaternion_factory.h>
#include <robowflex_dart/Object.h>


#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

using namespace robowflex;


bool plan_to_fold_arm(std::shared_ptr<darts::World> &world, darts::Window &window );


bool plan_to_move_robot (std::shared_ptr<darts::World> &world,darts::Window &window,Vector3d new_position);

bool plan_to_grasp(std::shared_ptr<darts::World> &world, darts::Window &window, Object &obj, int &surf_no, bool normals_help,
                   std::shared_ptr<darts::Robot> &robot_,Eigen::VectorXd start_config);

bool plan_to_move (std::shared_ptr<darts::World> &world,darts::Window &window,Object &obj, ActionR action_,
                            std::shared_ptr<darts::Robot> &robot_, std::shared_ptr<darts::Robot> &obj_robot_ );


void get_start_state (darts::PlanBuilder &builder_, std::vector<double> &config);

#endif //ROBOWFLEX_DART_PLANNINGFUNCTIONS_H

