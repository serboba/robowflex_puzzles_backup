//
// Created by serboba on 13.10.21.
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
#include <ompl/geometric/planners/sst/SST.h>

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
#include <robowflex_dart/point_collector.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";
static const std::string GROUP_X = "arm_with_x_move";

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             "/home/serboba/Desktop/blenderFLEX/fetch2.urdf",  //
                                             "/home/serboba/Desktop/blenderFLEX/fetch2.srdf");
    auto fetch_name = fetch_dart->getName();
    auto scene = std::make_shared<darts::Structure>("object");
    scene->addGround(-0.1);

    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    //world->addStructure(scene);
    //fetch_dart->setJoint("r_gripper_finger_joint", 0.01);
    //fetch_dart->setJoint("l_gripper_finger_joint", 0.01);
    //fetch_dart->setJoint("move_link_forward", 0.0);

    auto door_dart = darts::loadMoveItRobot("objects3",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/grasp.urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/grasp.srdf");
    world->addRobot(door_dart);
    darts::Window window(world);

    const auto &plan_to_grasp = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        builder.setStartConfigurationFromWorld();
        builder.initialize();

        ompl::base::PlannerStatus solved;
        while(true) {
            darts::TSR::Specification goal_spec2;  // IF GOAL NOT AS CONFIG
            goal_spec2.setFrame(fetch_name, "wrist_roll_link", "moving_link");
            goal_spec2.setPose(get_random_pose());

            goal_spec2.print(std::cout);
            auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);

            auto goal = builder.getGoalTSR(goal_tsr2);
            goal->setThreshold(0.0001);
            builder.setGoal(goal);

            //   std::this_thread::sleep_for(std::chrono::milliseconds(100000));

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
            //rrt->setRange(0.01);
            builder.ss->setPlanner(rrt);

            builder.setup();

            goal->startSampling();
            solved = builder.ss->solve(10.0);
            goal->stopSampling();
            if(solved){
                break;
            }
        }

        if (solved)
        {
                RBX_INFO("Found solution!");

                window.animatePath(builder, builder.getSolutionPath());

        }else{
            RBX_WARN("No solution found");
        }
    };

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        builder.addGroup(door_dart->getName(), "doorgr2");
/*
        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "moving_link");
        start_spec.setPose(
                0.6  ,          0    ,      1.3   ,
                0.707107  ,          0    , 0.707107   ,         0);
      //          0.600024   ,      0.23,          0.5  ,   0.500199  ,   0.499801 ,   -0.499801  ,  -0.500199);
        //        0.6   ,         0 ,         0.75  ,   0.707107  ,0.000281525 ,    0.707107, -0.000281556);
            //    0.3,  0.000238882  ,        0.5   ,         1 ,-0.000398163  ,          0  ,          0);
          //      0.600024   ,      0.03   ,       0.5    , 0.707107, -0.000281544,  0.000281544  ,  -0.707107);
        //start_spec.setPosition(0.78,-0.18,0.85);
        //start_spec.print(s0.7071 , 0.0 , 0.0 , -0.7071td::cout);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP_X);
        start_tsr.initialize();

        auto asd =  start_tsr.solveWorld();
*/

        builder.setStartConfigurationFromWorld();
        auto idk = builder.getStartConfiguration();

        darts::TSR::Specification con_spec;
        con_spec.setBase(door_dart->getName(),"door2");
        con_spec.setTarget(fetch_name,"wrist_roll_link");
        con_spec.setPoseFromWorld(world);

        auto rotation = con_spec.getRotation();

        auto cons_tsr = std::make_shared<darts::TSR>(world,con_spec);
        builder.addConstraint(cons_tsr);

        builder.initialize();

        darts::TSR::Specification goal_spec2;  // IF GOAL NOT AS CONFIG
        goal_spec2.setFrame(fetch_name,"wrist_roll_link","moving_link");
        goal_spec2.setPosition( 0.6  ,          0    ,      1.3);
        goal_spec2.setRotation(rotation);

        goal_spec2.print(std::cout);
        auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);

        auto goal = builder.getGoalTSR(goal_tsr2);


     //   auto goal = builder.getGoalConfiguration({-3.0});
        goal->setThreshold(0.10);

        builder.setGoal(goal);



    //   std::this_thread::sleep_for(std::chrono::milliseconds(100000));

        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        //rrt->setRange(0.01);
        builder.ss->setPlanner(rrt);

        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(600000.0);
        goal->stopSampling();

        if (solved)
        {
           // while(true){
            RBX_INFO("Found solution!");

                window.animatePath(builder, builder.getSolutionPath());
            //}
        }
        else
            RBX_WARN("No solution found");
    };

    window.run([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_grasp();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_pick();
    });

    return 0;
}
