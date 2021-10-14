//
// Created by serboba on 05.09.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

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

    fetch_dart->setJoint("r_gripper_finger_joint", 0.06);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.06);
    door_dart->setJoint("base_to_door",-1.57);

    darts::Window window(world);

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.addGroup(door_name,"dodoor");
        // 0.323309 -0.0526327 -0.144027 -0.849643 1.13029 1.06995 1.01299 2.15577 -1.57 END STATE

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.80 , -0.25, 0.75,  // goal pos
                           0.7071,0,0.7071,0);

        // 0.5,-0.5,0.5,0.5 false aber was ich will auch 0.5,0.5,0.5,-0.5
        // 0, 0.7071, 0.7071, 0 true
        //0,0.7071,0,-0.7071
        start_spec.print(std::cout);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        auto asd =  start_tsr.solveWorld();


        builder.setStartConfigurationFromWorld();
        auto idk = builder.getStartConfiguration();
        //    builder.addGroup(door_name,"dodoor");


        //    builder.setStartConfiguration({0.32, -0.05,-0.14, -0.84, 1.13, 1.06, 1.01, 2.15, -1.57});
        //    std::cout << "HOPPPP" << std::endl;
/*
        auto door_jt = door_dart->getGroupJoints("dodoor");
        builder.rspace->addGroupFromJoints(GROUP,door_jt);
*/
/*
        darts::TSR::Specification door_spec;
        door_spec.setFrame(door_name,"door","base_link");
        door_spec.setPoseFromWorld(world);
        door_spec.setRotation(1,0,0,0);

        std::cout << " yop" << std::endl;
        door_spec.print(std::cout);
        darts::TSR door_tsr(world, door_spec);
        door_tsr.useGroup("dodoor");
        door_tsr.initialize();
        auto aadsd =  door_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        auto idkk = builder.getStartConfiguration();

*/
        darts::TSR::Specification constr_spec2;

        constr_spec2.setBase(door_name,"door");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

         std::cout<< "CONSTRAINT 2 " << std::endl;
         constr_spec2.print(std::cout);

         darts::TSR::Specification constr_spec1;
         constr_spec1.setFrame(fetch_name,"wrist_roll_link","base_link");
         constr_spec1.setNoXPosTolerance();
         constr_spec1.setNoYPosTolerance();
         constr_spec1.setXRotTolerance(0.05);
         constr_spec1.setYRotTolerance(0.05);
         constr_spec1.setNoZRotTolerance();

        auto constr_tsr1 = std::make_shared<darts::TSR>(world,constr_spec1);
        builder.addConstraint(constr_tsr1);

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
        //    auto goal = builder.getGoalConfiguration({0.343, -0.252, -0.493, -0.003, 2.02, 0.06, 0.03, -1.50, -0.38});

        darts::TSR::Specification goal_spec1;  // IF GOAL NOT AS CONFIG
        goal_spec1.setFrame(door_name,"door","base_link");
        goal_spec1.setPose(0.4,-0.25,0.3,
                           1,0,0,0);
/*
        goal_spec2.setFrame(door_name,"door","base_link");
        goal_spec2.setPose(0.4,-0.25,0.35,
                           1,0,0,0);
*/
        goal_spec1.print(std::cout);
        auto goal_tsr1 = std::make_shared<darts::TSR>(world, goal_spec1);


        darts::TSR::Specification goal_spec2;  // IF GOAL NOT AS CONFIG
        goal_spec2.setFrame(fetch_name,"wrist_roll_link","base_link");
        goal_spec2.setPose(0.4,0.15,0.75,
                           0.5,-0.5,0.5,0.5);
/*
        goal_spec2.setFrame(door_name,"door","base_link");
        goal_spec2.setPose(0.4,-0.25,0.35,
                           1,0,0,0);
*/
        goal_spec2.print(std::cout);
        auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);

        //auto goal = builder.getGoalTSR(goal_tsr2);
        auto goal = builder.getGoalTSR({goal_tsr1,goal_tsr2});

        goal->setThreshold(0.05);
        builder.setGoal(goal);



        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        //rrt->setRange(1);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(600000.0);
        goal->stopSampling();

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