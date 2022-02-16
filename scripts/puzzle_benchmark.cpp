//
// Created by serboba on 28.01.22.
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

#include <robowflex_dart/gui.h>
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
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/PathGeometric.h>

 #include <robowflex_dart/RRTnew.h>
#include <robowflex_dart/urdf_read.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>


using namespace ompl;



void postRunEvent(const base::PlannerPtr & planner, tools::Benchmark::RunProperties & run)
{

    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->stopSampling();


//    planner->getProblemDefinition()->getSolutionPath()->as<ompl::geometric::PathGeometric>()->interpolate();
}


void preRunEvent(const base::PlannerPtr & planner){
    planner->getProblemDefinition()->getGoal()->as<base::GoalLazySamples>()->startSampling();

}

void benchmark(std::string robot_name,std::string urdf_name, std::string srdf_name){
    auto puzzle = robowflex::darts::loadMoveItRobot(robot_name,urdf_name,srdf_name);

    auto puzzle_name = puzzle->getName();
    auto world = std::make_shared<robowflex::darts::World>();
    world->addRobot(puzzle);

    robowflex::darts::PlanBuilder builder(world);


    URDF_IO input_(robot_name);

    for(std::string group : input_.group_names)
        builder.addGroup(puzzle_name,group);

    builder.setStartConfigurationFromWorld();
    builder.initialize();

    robowflex::darts::TSR::Specification goal_spec;
    goal_spec.setFrame(puzzle_name, "cube", "base_link");
    goal_spec.setPose(input_.goal_pose);            //  SET WANTED CUBE POSITION
    auto goal_tsr = std::make_shared<robowflex::darts::TSR>(world, goal_spec);
    auto goal = builder.getGoalTSR(goal_tsr);


    builder.setGoal(goal);
    builder.setGroupIndices(input_.group_indices);

    builder.space->setLongestValidSegmentFraction(0.01);
    builder.space->sanityChecks();
    builder.rspace->sanityChecks();

    builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,input_.group_indices));

    builder.setup();

    double runtime_limit = 60.0;
    double memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    int run_count = 100;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit,run_count);

    std::string b_name = "maze_Benchmarks";
    ompl::tools::Benchmark b(*builder.ss, b_name);



    // optionally set pre & pos run events
    b.setPreRunEvent([](const base::PlannerPtr &planner) { preRunEvent(planner); });
    b.setPostRunEvent(
            [](const base::PlannerPtr &planner, tools::Benchmark::RunProperties &run) { postRunEvent(planner, run); });


    auto rrt_new1 =std::make_shared<geometric::RRTnew>(builder.ss->getSpaceInformation(),input_.group_indices,false,true);
    rrt_new1->setRange(0.1);
    rrt_new1->setName("RRTnew_plus_iso");
    b.addPlanner(rrt_new1);


    auto rrt_new2 =std::make_shared<geometric::RRTnew>(builder.ss->getSpaceInformation(),input_.group_indices,false,false);
    rrt_new2->setRange(0.1);
    rrt_new2->setName("RRTnew");
    b.addPlanner(rrt_new2);

    //b.addPlanner(std::make_shared<geometric::RRTConnect>(builder.ss->getSpaceInformation()));

  /*  auto rrtconnect =std::make_shared<geometric::RRTConnect>(builder.ss->getSpaceInformation());
    rrtconnect->setRange(0.1);
    b.addPlanner(rrtconnect);

   // b.addPlanner(std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation()));
    auto rrtstar =std::make_shared<geometric::RRTstar>(builder.ss->getSpaceInformation());
    rrtstar->setRange(0.1);
    b.addPlanner(rrtstar);
*/
  /*
    //b.addPlanner(std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation()));
    auto lbtrrt =std::make_shared<geometric::LBTRRT>(builder.ss->getSpaceInformation());
    lbtrrt->setRange(0.1);
    b.addPlanner(lbtrrt);
*/
/*
    b.addPlanner(std::make_shared<geometric::ABITstar>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::AITstar>(builder.ss->getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::BITstar>(builder.ss->getSpaceInformation()));
*/
//    b.addPlanner(std::make_shared<geometric::BFMT>(builder.ss->getSpaceInformation()));



    b.addExperimentParameter("sampler_id", "INTEGER", "0"); // ?
    b.benchmark(request);

    std::string result_filename = "benchmark_results/"+robot_name;
    b.saveResultsToFile(result_filename.c_str());

}



int main(int argc, char **argv)
{

    benchmark("maze1",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze1.urdf",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze1.srdf");

    benchmark("maze2",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze2.urdf",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze2.srdf");


    benchmark("maze3",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze3.urdf",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze3.srdf");



    benchmark("maze_vertical",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze_vertical.urdf",
              "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/maze_vertical.srdf");

    return 0;
}
