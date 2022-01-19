/*********************************************************************
  * Rice University Software Distribution License
  *
  * Copyright (c) 2010, Rice University
  * All Rights Reserved.
  *
  * For a full description see the file named LICENSE.
  *
  *********************************************************************/

/* Author: Ioan Sucan */



//
// Created by serboba on 15.12.21.
//

//
// Created by serboba on 06.12.21.
//


//
// Created by serboba on 12.10.21.
//

//
// Created by serboba on 05.09.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/SimpleSetup.h>

#include <robowflex_library/builder.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>

#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/space.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/IsoManipulationOptimization.h>


#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>

#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>


#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>


using namespace ompl;

static const std::string GROUP = "arm_with_torso";



int main(int argc, char **argv)
{
    auto maze_dart = robowflex::darts::loadMoveItRobot("maze",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<robowflex::darts::World>();
    world->addRobot(maze_dart);


    robowflex::darts::PlanBuilder builder(world);


    builder.addGroup(maze_name, "cube_gr1");
    builder.addGroup(maze_name, "doorgr1");
    builder.addGroup(maze_name, "doorgr2");
    builder.addGroup(maze_name, "doorgr3");


    builder.initialize();
    base::ScopedState<base::RealVectorStateSpace> start(builder.info);
    start->values[0] = 0.0;
    start->values[1] = 0.0;
    start->values[2] = 0.0;
    start->values[3] = 0.0;
    start->values[4] = 0.0;

    base::ScopedState<base::RealVectorStateSpace> goal(builder.info);
    goal->values[0] = -0.05;
    goal->values[1] = 0.55;

    goal->values[2] = -0.51;
    goal->values[3] = 0.88;
    goal->values[4] = 1.30;

    builder.ss->setStartState(start);
    builder.ss->setGoalState(goal);

    //auto goal = builder.getGoalConfiguration({-0.51,0.88,1.30 ,-0.05,0.55});
    //builder.setGoal(goal);
    builder.space->sanityChecks();
    

    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info));

    //builder.setup();


    double runtime_limit = 60.0;
    double memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    int run_count = 100;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit,run_count);
    std::string b_name = std::string("maze_3doors_benchmark");
    ompl::tools::Benchmark b(*builder.ss, b_name);

    // optionally set pre & pos run events
    //b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
   // b.setPostRunEvent(
    //        [](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });

    b.addPlanner(std::make_shared<geometric::RRTConnect>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRT>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));

    b.addPlanner(std::make_shared<geometric::BITstar>(builder.ss->getSpaceInformation()));

    b.addPlanner(std::make_shared<geometric::FMT>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::BFMT>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::KPIECE1>(builder.ss->getSpaceInformation()));

        // run all planners with a uniform valid state sampler on the benchmark problem
        builder.ss->getSpaceInformation()->setValidStateSamplerAllocator(
                [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr {
                    return std::make_shared<base::UniformValidStateSampler>(si);
                });
        b.addExperimentParameter("sampler_id", "INTEGER", "0");

        b.benchmark(request);
        b.saveResultsToFile("maze_log2");

    return 0;
}