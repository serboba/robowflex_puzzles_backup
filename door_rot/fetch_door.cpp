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

    scene->fromYAMLFile("/home/serboba/Desktop/blenderfiles/door_test/door_push.yml");


    //
    // Convert to Dart
    //
    auto fetch_dart = std::make_shared<darts::Robot>(fetch);
    auto fetch_name = fetch_dart->getName();
    auto scene_dart = std::make_shared<darts::Structure>("scene", scene);

    // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addStructure(scene_dart);

    fetch_dart->setJoint("r_gripper_finger_joint", 0.05);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.05);

    darts::Window window(world);

    const auto &plan_to_grip_door = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.33, 0.0, 0.7,  // goal pos 72+20
                           0.0, 0.0, -1.0, 0.0);
        start_spec.set


        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.53, -0.38, 0.7,  // starting pos 72+60
                          0.0, -0.71, -0.71, 0.0);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

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

    const auto &plan_to_grasp_door = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification constraint_spec;
        constraint_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        constraint_spec.setPose(0.53, -0.38, 0.7,  // start pos
                                0.0,-0.71, -0.71, 0.0);
        constraint_spec.setNoZPosTolerance();

        // ?????????????????????00
        //constraint_spec.setU

        auto constraint_tsr = std::make_shared<darts::TSR>(world, constraint_spec);
        builder.addConstraint(constraint_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.53, -0.38, 0.7,  // start pos
                          0.0, -0.71, -0.71, 0.0);


        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

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

    const auto &plan_to_rotate = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification constraint_spec;
        constraint_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        constraint_spec.setPose(0.53, -0.38, 0.7, // start pos
                                0.0, -0.71, -0.71, 0.0);
//table edges -0.45????? 18m in blender x 0.025 dim in fetch yml?
        //constraint_spec.setNoXPosTolerance();
        //constraint_spec.setNoYPosTolerance();
        auto constraint_tsr = std::make_shared<darts::TSR>(world,constraint_spec);
        builder.addConstraint(constraint_tsr);


        builder.initialize();
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.38, 0.0, 0.7,  // goal pos
                          0.0, -1.0, 0.0, 0.0);


        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(500.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while(true){
            if (solved)
            {
                RBX_INFO("Found solution!");
                window.animatePath(builder, builder.getSolutionPath());
            }
            else
                RBX_WARN("No solution found");
        }
    };


    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.15, -0.65, 0.77,  // goal pos 72+20
                           0.5, -0.5, 0.5, 0.5);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.73, 0.1, 1.17,  // starting pos 72+60
                          0.5, -0.5, 0.5, 0.5);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

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

    const auto &plan_to_grasp = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification constraint_spec;
        constraint_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        constraint_spec.setPose(0.73, 0.1, 0.77,  // start pos
                                0.5, -0.5, 0.5, 0.5);
        constraint_spec.setNoZPosTolerance();

        // ?????????????????????
        //constraint_spec.setU

        auto constraint_tsr = std::make_shared<darts::TSR>(world, constraint_spec);
        builder.addConstraint(constraint_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.73, 0.1, 0.77,  // start pos
                          0.5, -0.5, 0.5, 0.5);


        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

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

    const auto &plan_to_place = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification constraint_spec;
        constraint_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        constraint_spec.setPose(0.73, 0.1, 0.77, // start pos
                                0.5, -0.5, 0.5, 0.5);
//table edges -0.45????? 18m in blender x 0.025 dim in fetch yml?
        constraint_spec.setNoXPosTolerance();
        constraint_spec.setNoYPosTolerance();
        auto constraint_tsr = std::make_shared<darts::TSR>(world,constraint_spec);
        builder.addConstraint(constraint_tsr);


        builder.initialize();
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.15, -0.65, 0.77,  // goal pos
                          0.5, -0.5, 0.5, 0.5);


        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(500.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        while(true){
            if (solved)
            {
                RBX_INFO("Found solution!");
                window.animatePath(builder, builder.getSolutionPath());
            }
            else
                RBX_WARN("No solution found");
        }
    };

    window.run([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        //plan to pos?
        //plan to grip
        //plan to push
        //
        //plan to pick, .. grasp, place
        plan_to_grip_door();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_grasp_door();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_rotate();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
/*
        plan_to_pick();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_grasp();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
*/
        auto *door = scene_dart->getFrame("door");
        fetch_dart->reparentFreeFrame(door, "wrist_roll_link");

        plan_to_place();
    });

    return 0;
}