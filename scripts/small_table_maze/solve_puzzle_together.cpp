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
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
//#include <ompl/geometric/planners/rrt/RRT.h>
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

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);


    auto maze_dart = darts::loadMoveItRobot("maze",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(maze_dart);

    darts::Window window(world);

    const auto &plan_solution_all = [&]() {
        darts::PlanBuilder builder(world);
        std::vector<double> goal_state_vector;

        builder.addGroup(maze_name, "cube_gr1"); // ADD ALL GROUPS THAT ARE NEEDED
        builder.addGroup(maze_name, "cube_gr2");
        builder.addGroup(maze_name, "doorgr1");
        builder.addGroup(maze_name, "doorgr2");
        builder.addGroup(maze_name, "doorgr3");


        builder.setStartConfigurationFromWorld();
        //auto idk = builder.getStartConfiguration();
        builder.initialize();


        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(maze_name, "cube1", "base_link");
        goal_spec.setPose(0.5, -0.25, 0.75, 1, 0 ,0 ,0);            //  SET WANTED CUBE POSITION
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);

        //auto goal = builder.getGoalConfiguration(goal_state_vector);
        goal->setThreshold(0.01);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(0.1);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(600.0);
        goal->stopSampling();
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath(),3,15);
          //  std::ofstream fs("values.txt");
          //  builder.getSolutionPath().printAsMatrix(fs);

        }
        else
            RBX_WARN("No solution found");
    };


    window.run([&] {
        std::vector<std::pair<std::string,double>> assigned;
        //solve_backtracking(world, assigned,window);
        // std:: cout << backtracking(assigned,world,window) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));

        plan_solution_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(200000));
        // move_cube(assigned,world,window);
    });

    return 0;
}

