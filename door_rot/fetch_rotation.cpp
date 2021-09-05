//
// Created by serboba on 05.09.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

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

    //
    // Standard Robowflex setup
    // Create the default Fetch robot and scene.
    //

    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();
    auto scene = std::make_shared<Scene>(fetch);

    auto door = darts::Robot("myfirst");
    door.loadURDF("/home/serboba/Desktop/blenderFLEX/hello.urdf");
    door.loadSRDF("/home/serboba/Desktop/blenderFLEX/hello.srdf");


    auto fetch_dart = std::make_shared<darts::Robot>(fetch);
    auto fetch_name = fetch_dart->getName();

    auto door_dart = std::make_shared<darts::Robot>(door);
    auto door_name = door_dart->getName();

    auto scene_dart = std::make_shared<darts::Structure>("scene", scene);

    // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(door_dart);
    world->addStructure(scene_dart);
    // scene_dart->updateCollisionObject("myfirst",door_dart);

    fetch_dart->setJoint("r_gripper_finger_joint", 0.05);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.05);
    //door_dart->setJoint("base_to_door",0.0);

    darts::Window window(world);

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.40, 0.13, 0.85,  // starting pos 72+60
                           0.5, -0.5, 0.5, 0.5);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);

        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();

        auto door_jt = door_dart->getGroupJoints("dodoor");
        builder.rspace->addGroupFromJoints(GROUP,door_jt);

        darts::TSR::Specification constr_spec2;
        constr_spec2.setBase(door_name,"door");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.78 , -0.25, 0.85,  // goal pos
                          0.0, 0.71, 0.0, -0.71);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

//        auto testaq = builder.constraint->getSet();
 //       bool amk = testaq->solveWorld();
   //     builder.ss->print();
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
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