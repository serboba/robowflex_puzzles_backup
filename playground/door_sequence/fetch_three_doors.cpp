//
// Created by serboba on 14.10.21.
//

//
// Created by serboba on 04.09.21.
//

//
// Created by serboba on 05.09.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>

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

    //
    // Standard Robowflex setup
    // Create the default Fetch robot and scene.
    //

    auto fetch = std::make_shared<FetchRobot>();
    fetch->initialize();
    auto scene = std::make_shared<Scene>(fetch);

    auto door_dart = darts::loadMoveItRobot("objects3",
                                       "/home/serboba/Desktop/blenderFLEX/doors.urdf",
                                       "/home/serboba/Desktop/blenderFLEX/doors.srdf");


    auto fetch_dart = std::make_shared<darts::Robot>(fetch);
    auto fetch_name = fetch_dart->getName();

    auto door_name = door_dart->getName();

    // Setup world
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(door_dart);

    fetch_dart->setJoint("r_gripper_finger_joint", 0.03);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.03);

    Rotation_Helper rotation_helper;

    //door_dart->setJoint("base_to_door",0.0);
    const Eigen::Vector3d start_position_door1 = {0.55,-0.13,0.62};
    const Eigen::Quaterniond start_rotation_door1 {0.5, -0.5, 0.5, 0.5};

    const Eigen::Vector3d end_position_door1 = {0.72,-0.3,0.62};
 //   const Eigen::Quaterniond end_rotation_door1 {0.7071,0,0.7071,0};

    const Eigen::Vector3d start_position_door2 = {0.48, 0.0,0.62};
    const Eigen::Quaterniond start_rotation_door2= {0.0 , 0.7071 , 0.0 , -0.7071};

    const Eigen::Vector3d end_position_door2 = {0.65, -0.17,0.62};
 //   const Eigen::Quaterniond end_rotation_door2= {0.7071,0,0.7071,0};


    const Eigen::Vector3d start_position_door3 = {0.55, 0.13,0.62};
    const Eigen::Quaterniond start_rotation_door3 = {0.5 , 0.5 , 0.5 , -0.5};

    const Eigen::Vector3d end_position_door3 = {0.38, 0.30,0.62};
    //const Eigen::Quaterniond end_rotation_door3 = {0.5, -0.5, 0.5, -0.5};

    std::list<ompl::geometric::PathGeometric> path_results;


    darts::Window window(world);
    const auto &plan_to_grasp1 = [&](Eigen::Vector3d start_position, Eigen::Quaterniond start_rotation){

        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        start_position.z()+= 0.2;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(start_position,  // goal pos
                          start_rotation);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);


        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
        rrt->setRange(1);
        builder.ss->setPlanner(rrt);

        builder.setup();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60000.0);
        goal->stopSampling();

        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            path_results.push_back(builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution found");

    };

    const auto &grasp = [&](){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification con_spec;
        con_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        con_spec.setPoseFromWorld(world);
        con_spec.setNoZPosTolerance();

        auto con_tsr = std::make_shared<darts::TSR>(world, con_spec);
        builder.addConstraint(con_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPoseFromWorld(world);
        auto pos = goal_spec.getPosition();
        auto rot = goal_spec.getRotation();
        pos.z() -= 0.20;

        goal_spec.setPose(pos,rot);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);

        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
        rrt->setRange(2);
        builder.ss->setPlanner(rrt);

        builder.setup();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(600000.0);
        goal->stopSampling();

        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            path_results.push_back(builder.getSolutionPath());
        }
        else
            RBX_WARN("No solution found??");

    };

    const auto &plan_to_rotate = [&](int direction, std::string joint_name, std::string group_name,
                                    Eigen::Vector3d end_position) {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);
        builder.addGroup(door_name,group_name);

        builder.setStartConfigurationFromWorld();


        darts::TSR::Specification posspec;
        posspec.setFrame(fetch_name,"wrist_roll_link","base_link");
        posspec.setPoseFromWorld(world);
        auto rot = posspec.getRotation();



        darts::TSR::Specification constr_spec2;
        constr_spec2.setBase(door_name,joint_name);
        constr_spec2.setTarget(fetch_name,"wrist_roll_link");
        constr_spec2.setPoseFromWorld(world);


        auto constraint_tsr2 = std::make_shared<darts::TSR>(world,constr_spec2);
        builder.addConstraint(constraint_tsr2);

        builder.initialize();

        Eigen::Quaterniond new_rotation;

        if(direction== 0)
            new_rotation = rotation_helper.rotateRight(rot);
        else
            new_rotation = rotation_helper.rotateLeft(rot);


        std::cout << "OLD ROTATION: " << std::endl;
        std::cout << rot.w() << std::endl;
        std::cout << rot.x() << std::endl;
        std::cout << rot.y() << std::endl;
        std::cout << rot.z() << std::endl;

        std::cout << "NEW ROTATION: " << std::endl;
        std::cout << new_rotation.w() << std::endl;
        std::cout << new_rotation.x() << std::endl;
        std::cout << new_rotation.y() << std::endl;
        std::cout << new_rotation.z() << std::endl;



        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(end_position,  // goal pos
                          new_rotation);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        goal->setThreshold(0.01);
        builder.setGoal(goal);


        auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        builder.ss->setPlanner(rrt);

        builder.setup();


        //9 builder.ss->print();
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60000.0);
        goal->stopSampling();

        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            path_results.push_back(builder.getSolutionPath());

        }
        else
            RBX_WARN("No solution found");
    };

    const auto &detach_from_door = [&](){

        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name,GROUP);

        builder.setStartConfigurationFromWorld();

        darts::TSR::Specification con_spec;
        con_spec.setFrame(fetch_name,"wrist_roll_link","base_link");
        con_spec.setPoseFromWorld(world);
        con_spec.setNoZPosTolerance();

        auto con_tsr = std::make_shared<darts::TSR>(world,con_spec);
        builder.addConstraint(con_tsr);


        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPoseFromWorld(world);
        auto pos = goal_spec.getPosition();
        auto rot = goal_spec.getRotation();
        pos.z()+= 0.2;
        goal_spec.setPose(pos,rot);

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
        builder.ss->setPlanner(rrt);

        builder.setup();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        if (solved)
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            path_results.push_back(builder.getSolutionPath());

        }
        else
            RBX_WARN("No solution found");

    };



    window.run([&] {
        // direction parameter 0 = right, 1 left


        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        plan_to_grasp1(start_position_door1,start_rotation_door1);
        grasp();
        plan_to_rotate(0,"door1","doorgr1", end_position_door1);
        detach_from_door();


        plan_to_grasp1(start_position_door2,start_rotation_door2);
        grasp();
        plan_to_rotate(1,"door2", "doorgr2", end_position_door2);
        detach_from_door();

        plan_to_grasp1(start_position_door3,start_rotation_door3);
        grasp();
        plan_to_rotate(0,"door3", "doorgr3", end_position_door3);
        detach_from_door();

        std::this_thread::sleep_for(std::chrono::milliseconds(10000));




    });

    return 0;
}
