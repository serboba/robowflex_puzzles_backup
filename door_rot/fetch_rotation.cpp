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
#include <robowflex_dart/rotation_helper.h>

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

    Rotation_Helper rotationHelper;

    darts::Window window(world);

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.addGroup(door_name,"dodoor");

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.80 , -0.25, 0.75,  // goal pos
                           0.7071,0,0.7071,0);

        start_spec.print(std::cout);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        auto tst =  start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification constr_spec2;

        constr_spec2.setBase(door_name,"door");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

        builder.initialize();

        darts::TSR::Specification goal_spec;  // IF GOAL NOT AS CONFIG
        goal_spec.setFrame(fetch_name,"wrist_roll_link","base_link");
        goal_spec.setPosition(0.4,0.15,0.75);
        goal_spec.setRotation(rotationHelper.rotateLeft({0.7071,0,0.7071,0}));

        goal_spec.print(std::cout);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);

        auto goal = builder.getGoalTSR(goal_tsr);

        goal->setThreshold(0.01);
        builder.setGoal(goal);


        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto kpiece = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        builder.ss->setPlanner(kpiece);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
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