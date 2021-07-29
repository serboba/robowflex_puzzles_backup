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
    door.loadURDF("/home/serboba/Desktop/blenderfiles/hello.urdf");
    door.loadSRDF("/home/serboba/Desktop/blenderfiles/hello.srdf");

    //door.setDof(0,0.0); x rot
    //door.setDof(1,0.5); y
    door.setDof(2,0.0);
    door.setDof(3,0.4);
    door.setDof(4, -0.2); // y -axis -0.2
    door.setDof(5, 0.0);

    //
    // Convert to Dart
    //
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


    fetch_dart->setJoint("r_gripper_finger_joint", 0.03);
    fetch_dart->setJoint("l_gripper_finger_joint", 0.03);
    //door_dart->setJoint("base_to_door",0.0);



    darts::Window window(world);

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP);


        darts::TSR::Specification start_spec;
        start_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        start_spec.setPose(0.73, -0.21, 0.76,  // goal pos 72+20
                           0.0, 0.71, 0.0, -0.71);

        darts::TSR start_tsr(world, start_spec);
        start_tsr.useGroup(GROUP);
        start_tsr.initialize();
        start_tsr.solveWorld();

        builder.setStartConfigurationFromWorld();
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.40, 0.27, 0.90,  // starting pos 72+60
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
        constraint_spec.setPose(0.40, 0.27, 0.76,  // starting pos
                                0.5, -0.5, 0.5, 0.5);
        constraint_spec.setNoZPosTolerance();

        // ?????????????????????
        //constraint_spec.setU

        auto constraint_tsr = std::make_shared<darts::TSR>(world, constraint_spec);
        builder.addConstraint(constraint_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.40, 0.27, 0.76,  // starting pos
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
        //builder.addGroup("myfirst","dodoor");
        RBX_INFO("PROBLEM1");
        builder.addGroup(door_name,"base_to_door");
        builder.setStartConfigurationFromWorld();
        RBX_INFO("PROBLEM2");


        darts::TSR::Specification group_spec;
        group_spec.setTarget(fetch_name,"door");
        group_spec.setBase(fetch_name,"wrist_roll_link");
        group_spec.setPoseFromWorld(world);
        auto gr_spec = std::make_shared<darts::TSR>(world,group_spec);
        builder.addConstraint(gr_spec);


        darts::TSR::Specification constr_spec;
        constr_spec.setFrame(fetch_name,"wrist_roll_link", "base_link");
        constr_spec.setPose(0.40, 0.27, 0.76,  // starting pos
                            0.5, -0.5, 0.5, 0.5);

        auto constraint_tsr = std::make_shared<darts::TSR>(world,constr_spec);
        builder.addConstraint(constraint_tsr);
        RBX_INFO("PROBLEM6");

        builder.initialize();
        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name, "wrist_roll_link", "base_link");
        goal_spec.setPose(0.73, -0.21, 0.76,  // goal pos
                          0.0, 0.71, 0.0, -0.71);


        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);

        auto goal = builder.getGoalTSR(goal_tsr);
        //hier problem
        builder.setGoal(goal);
        RBX_INFO("PROBLEM7");
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
        plan_to_pick();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_to_grasp();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        auto *cube = door_dart->getFrame("base_link1");
        fetch_dart->reparentFreeFrame(cube, "wrist_roll_link");
        RBX_INFO("DURCH2");
        plan_to_place();
    });

    return 0;
}