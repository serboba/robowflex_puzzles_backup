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

#include <robowflex_dart/RRTnew.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>



#include <algorithm>


#include <robowflex_library/builder.h>
#include <robowflex_library/detail/fetch.h>
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
#include <robowflex_dart/solution_parser.h>
#include <robowflex_dart/IsoManipulationOptimization.h>
#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/urdf_read.h>


using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    std::string env_name = "maze3";

    auto maze_dart = darts::loadMoveItRobot(env_name,
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/" + env_name + ".urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/" + env_name + ".srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(maze_dart);

    darts::Window window(world);



    const auto &plan_solution_all = [&]() {

        URDF_IO input_(env_name);

        darts::PlanBuilder builder(world,input_.group_indices); // using my statespace


        for(std::string group : input_.group_names) {
            builder.addGroup(maze_name,group);
        }
        // ADD ALL GROUPS THAT ARE NEEDED

        builder.setStartConfigurationFromWorld();



        builder.initialize();


        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(maze_name, "link_0", "base_link");
        goal_spec.setPose(input_.goal_pose);
        goal_spec.print(std::cout);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);

        builder.setGoal(goal);


        builder.ss->setOptimizationObjective(std::make_shared<ompl::base::IsoManipulationOptimization>(builder.info,input_.group_indices));
         auto planner = std::make_shared<ompl::geometric::RRTnew>(builder.info,input_.group_indices,false,true); // last parameter is state isolation
      //  auto planner = std::make_shared<ompl::geometric::RRTstar>(builder.info);
      //  auto planner = std::make_shared<ompl::geometric::BITstar>(builder.info);
       // auto planner = std::make_shared<ompl::geometric::RRTConnect>(builder.info,false);

        builder.ss->setPlanner(planner);
        builder.setup();

        builder.space->sanityChecks();
        builder.rspace->sanityChecks();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(30);
        goal->stopSampling();



        if (solved)
        {
            ompl::geometric::PathGeometric path(builder.getSolutionPath(false,false));

            //path.interpolate() if not rrtnew

            std::cout << "path" << std::endl;
            std::cout << path.getStateCount() << std::endl;
            std::string file_name = "path_result/"+env_name + ".txt";
            std::ofstream fs(file_name);
            path.printAsMatrix(fs);

           window.animatePath(builder, path,5,1);


       }
        else
            RBX_WARN("No solution found");
    };


    window.run([&] {

     //   std::this_thread::sleep_for(std::chrono::milliseconds(200000));
        plan_solution_all();

    });

    return 0;
}


