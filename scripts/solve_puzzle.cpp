//
// Created by serboba on 23.12.21.
//

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
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <robowflex_dart/RRTnew.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
//#include <robowflex_dart/IsoManipulationStateSpace.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <algorithm>
#include <robowflex_dart/IsoManipulationOptimization.h>

#include <robowflex_library/builder.h>

#include <robowflex_library/detail/fetch.h>
#include <robowflex_library/log.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/tf.h>
#include <robowflex_library/util.h>
#include <python2.7/Python.h>

#include <robowflex_dart/gui.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>
#include <robowflex_dart/solution_parser.h>
#include <robowflex_dart/urdf_read.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{

    // Startup ROS
    ROS ros(argc, argv);
    // todo get filename etc as input

    auto maze_dart = darts::loadMoveItRobot("maze",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze_vertical.urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze_vertical.srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(maze_dart);


    darts::Window window(world);

    const auto &plan_solution_all = [&]() {
        darts::PlanBuilder builder(world);

        URDF_IO input_("maze_vertical");

        for(std::string group : input_.group_names)
            builder.addGroup(maze_name,group);
        
        /*
        builder.addGroup(maze_name, "doorgr1");
        //builder.addGroup(maze_name, "doorgr2");
        builder.addGroup(maze_name, "cube_gr1");
        builder.addGroup(maze_name, "doorgr3");
        //builder.addGroup(maze_name, "doorgr4");
        */
        builder.setStartConfigurationFromWorld();


        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(maze_name, "cube", "base_link");
        goal_spec.setPose(input_.goal_pose);            //  SET WANTED CUBE POSITION
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);

        builder.setGoal(goal);

        std::vector<int> gr_ind = input_.group_indices.at(0);
        
        auto planner = std::make_shared<ompl::geometric::RRTnew>(builder.info,gr_ind,false);

        planner->setRange(0.1);
        builder.ss->setPlanner(planner);

        builder.setup();


        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        if (solved)
        {
            ompl::geometric::PathGeometric path = builder.getSolutionPath(false,false);

            std::cout << "path1" << std::endl;
            std::cout << path.getStateCount() << std::endl;
            RBX_INFO("Found solution!");
            
            std::cout << "start" << std::endl;
            std::ofstream fs("mazesolution.txt");
            path.printAsMatrix(fs);
            std::cout << "end" << std::endl;


            window.animatePath(builder, path,2,5);
           }
        else
            RBX_WARN("No solution found");
    };


    window.run([&] {
        
        plan_solution_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(200000));

    });

    return 0;
}

