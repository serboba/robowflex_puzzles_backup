//
// Created by serboba on 05.09.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/rrt/RRT.h>

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

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             "/home/serboba/Desktop/blenderFLEX/fetch2.urdf",  //
                                             "/home/serboba/Desktop/blenderFLEX/fetch2.srdf");

    auto door_dart = darts::loadMoveItRobot("myfirst",
                                        "/home/serboba/Desktop/blenderFLEX/hello.urdf",
                                        "/home/serboba/Desktop/blenderFLEX/hello.srdf");

    auto fetch_name = fetch_dart->getName();
    auto door_name = door_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(door_dart);

    fetch_dart->setJoint("r_gripper_finger_joint", 0.05);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.05);
    door_dart->setJoint("base_to_door",-1.57);

    darts::Window window(world);

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.addGroup(door_name,"dodoor");
        // 0.323309 -0.0526327 -0.144027 -0.849643 1.13029 1.06995 1.01299 2.15577 -1.57 END STATE

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.55 , -0.25, 0.85,  // goal pos
                           0.7071, 0, 0.7071, 0);


        start_spec.print(std::cout);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);

        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
   //    builder.setStartConfiguration({0.32, -0.05,-0.14, -0.84, 1.13, 1.06, 1.01, 2.15, -1.57});
   //    auto idk = builder.getStartConfiguration();
   //    std::cout << "HOPPPP" << std::endl;

        auto door_jt = door_dart->getGroupJoints("dodoor");
        builder.rspace->addGroupFromJoints(GROUP,door_jt);


        darts::TSR::Specification constr_spec2;

        constr_spec2.setBase(door_name,"door");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

       // std::cout<< "CONSTRAINT 1 " << std::endl;
       // constr_spec2.print(std::cout);

        builder.initialize();

/*      darts::TSR::Specification goal_spec;  // IF GOAL NOT AS CONFIG
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.45 , 0.00, 0.85,  // goal pos
                          0.5,-0.5,0.5,0.5);

        std::cout <<   "GOAL 1 " << std::endl;
        goal_spec.print(std::cout);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
  */

// END STATE ORIGINAL      // 0.323309 -0.0526327 -0.144027 -0.849643 1.13029 1.06995 1.01299 2.15577 -1.57 END STATE
// end state 2 ////          0.38615 ,0.0133584, -0.217327, -0.19957, 2.07972, -0.611774,-0.345477,-1.01676 ,0.0

// auto goal = builder.getGoalConfiguration({0.38 ,0.01, -0.21, -0.2, 2.08, -0.61,-0.34,-1.01 ,0.0});
        auto goal = builder.getGoalConfiguration({0.343, -0.252, -0.493, -0.003, 2.02, 0.06, 0.03, -1.50, -0.38});
        builder.setGoal(goal);


        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

      // goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(600000.0);
       //goal->stopSampling();

        if (solved)
        {
            RBX_INFO("Found solution!");
            while(true){
                window.animatePath(builder, builder.getSolutionPath());
            }
        }
        else
            RBX_WARN("No solution found");
    };

    window.run([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_pick();
    });

    return 0;
}