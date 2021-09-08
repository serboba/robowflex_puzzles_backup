//
// Created by serboba on 04.09.21.
//

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

    auto door = darts::Robot("objects3");
    door.loadURDF("/home/serboba/Desktop/blenderFLEX/doors.urdf");
    door.loadSRDF("/home/serboba/Desktop/blenderFLEX/doors.srdf");


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

    fetch_dart->setJoint("r_gripper_finger_joint", 0.02);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.02);
    //door_dart->setJoint("base_to_door",0.0);
    const Eigen::Vector3d start_position_door1 = {0.55,-0.13,0.52};
    const Eigen::Quaterniond start_rotation_door1 {0.5, -0.5, 0.5, 0.5};

    const Eigen::Vector3d end_position_door1 = {0.72,-0.3,0.52};
    const Eigen::Quaterniond end_rotation_door1 {0.0, 0.71, 0.0, -0.71};

    const Eigen::Vector3d start_position_door2 = {0.38, 0.0,0.52};
    const Eigen::Quaterniond start_rotation_door2= {0.0, 0.71, 0.0, -0.71};

    const Eigen::Vector3d end_position_door2 = {0.72, 0.0,0.52};
    const Eigen::Quaterniond end_rotation_door2= {0.5, -0.5, 0.5, 0.5};


    const Eigen::Vector3d start_position_door3 = {0.55, 0.13,0.52};
    const Eigen::Quaterniond start_rotation_door3 = {0.5, -0.5, 0.5, 0.5};

    const Eigen::Vector3d end_position_door3 = {0.55, 0.30,0.52};
    const Eigen::Quaterniond end_rotation_door3 = {0.5, -0.5, 0.5, 0.5};


    darts::Window window(world);
    const auto &plan_to_grasp1 = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name,"wrist_roll_link", "base_link");
        start_spec.setPoseFromWorld(world);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        auto grasp_pos1 = start_position_door1;
        grasp_pos1.z() = grasp_pos1.z()+0.2;
        goal_spec.setPose(grasp_pos1,  // goal pos
                          start_rotation_door1);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

    const auto &grasp_1 = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPoseFromWorld(world);
        start_spec.setNoZPosTolerance();

        auto start_tsr = std::make_shared<darts::TSR>(world, start_spec);
        builder.addConstraint(start_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPosition(start_position_door1);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(10.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution found??");

    };

    const auto &plan_to_rotate1 = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPosition(start_position_door1);
        start_spec.setRotation(start_rotation_door1);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        builder.setStartConfigurationFromWorld();

        auto door_jt = door_dart->getGroupJoints("doorgr1");
        builder.rspace->addGroupFromJoints(GROUP,door_jt);

        darts::TSR::Specification constr_spec2;
        constr_spec2.setBase(door_name,"door1");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

        builder.initialize();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(end_position_door1,  // goal pos
                          end_rotation_door1);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();


        //9 builder.ss->print();
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(1.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution foundAAAAAAAAA");
    };

    const auto &detach_from_door = [&](){

        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        //start_spec.setPosition(end_position_door1);
        //start_spec.setRotation(end_rotation_door1);
        start_spec.setPoseFromWorld(world); // reached end pos change after segfault bug fixed delete 2 lines above
        start_spec.setNoZPosTolerance();
        auto curr_pos = start_spec.getPosition();
        auto curr_rot = start_spec.getRotation();
        auto start_tsr = std::make_shared<darts::TSR>(world, start_spec);
        builder.addConstraint(start_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        //auto detach_pos = end_position_door1; // for later detach from the object go up
        //detach_pos.z() += 0.20;
        curr_pos.z() += 0.20; //temp fix

        //goal_spec.setPosition(detach_pos);
        goal_spec.setPosition(curr_pos);
        //goal_spec.setRotation()
        //goal_spec.setRotation(end_rotation_door1);
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
            RBX_WARN("No solution foundqqq");

    };

    const auto &plan_to_grasp2 = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name,"wrist_roll_link", "base_link");
        start_spec.setPoseFromWorld(world);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        auto grasp_pos2 = start_position_door2;
        grasp_pos2.z() = grasp_pos2.z()+0.2;
        goal_spec.setPose(grasp_pos2,  // goal pos
                          start_rotation_door2);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

    const auto &grasp_2 = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPoseFromWorld(world);
        start_spec.setNoZPosTolerance();

        auto start_tsr = std::make_shared<darts::TSR>(world, start_spec);
        builder.addConstraint(start_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPosition(start_position_door2);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(10.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution found??");

    };

    const auto &plan_to_rotate2 = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(start_position_door2, start_rotation_door1);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        builder.setStartConfigurationFromWorld();

        auto door_jt = door_dart->getGroupJoints("doorgr2");
        builder.rspace->addGroupFromJoints(GROUP,door_jt);

        darts::TSR::Specification constr_spec2;
        constr_spec2.setBase(door_name,"door2");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

        builder.initialize();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        //goal_spec.setPose(end_position_door2, end_rotation_door2); // goal pos
        goal_spec.setPosition(end_position_door2);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        //9 builder.ss->print();
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(1.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution foundAAAAAAAAA");
    };

    const auto &plan_to_grasp3 = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name,"wrist_roll_link", "base_link");
        start_spec.setPoseFromWorld(world);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        auto grasp_pos3 = start_position_door3;
        grasp_pos3.z() = grasp_pos3.z()+0.2;
        goal_spec.setPose(grasp_pos3,  // goal pos
                          start_rotation_door3);

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

    const auto &grasp_3 = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPoseFromWorld(world);
        start_spec.setNoZPosTolerance();

        auto start_tsr = std::make_shared<darts::TSR>(world, start_spec);
        builder.addConstraint(start_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPosition(start_position_door3);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(10.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution found??");

    };

    const auto &plan_to_rotate3 = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);

        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(start_position_door3, start_rotation_door3);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        builder.setStartConfigurationFromWorld();

        auto door_jt = door_dart->getGroupJoints("doorgr3");
        builder.rspace->addGroupFromJoints(GROUP,door_jt);

        darts::TSR::Specification constr_spec2;
        constr_spec2.setBase(door_name,"door3");
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);

        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

        builder.initialize();

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        //goal_spec.setPose(end_position_door2, end_rotation_door2); // goal pos
        goal_spec.setPosition(end_position_door3);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info, true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        //9 builder.ss->print();
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(1.0);
        goal->stopSampling();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution foundAAAAAAAAA");
    };



    window.run([&] {
        plan_to_grasp1();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        grasp_1();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_rotate1();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        detach_from_door();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_grasp2();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        grasp_2();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_rotate2();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        detach_from_door();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_grasp3();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        grasp_3();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_rotate3();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        detach_from_door();

    });

    return 0;
}