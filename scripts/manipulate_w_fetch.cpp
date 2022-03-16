//
// Created by serboba on 20.01.22.
//

//
// Created by serboba on 03.12.21.
//

//
// Created by serboba on 13.10.21.
//

//
// Created by serboba on 05.09.21.
//

//
// Created by serboba 19.11.21.
//

#include <chrono>
#include <thread>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
//#include <ompl/geometric/planners/rrt/RRT.h>
//#include <ompl/geometric/planners/sst/SST.h>
#include <robowflex_dart/ActionRobot.h>
#include <robowflex_dart/urdf_read.h>

#include <robowflex_library/builder.h>
#include <robowflex_library/log.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/util.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <robowflex_library/class_forward.h>
#include <robowflex_dart/gui.h>
#include <robowflex_dart/planning.h>
#include <robowflex_dart/robot.h>
#include <robowflex_dart/tsr.h>
#include <robowflex_dart/world.h>

#include <robowflex_dart/point_collector.h>
#include <robowflex_dart/conversion_functions.h>
#include <robowflex_dart/quaternion_factory.h>
#include <robowflex_dart/Object.h>
#include <robowflex_dart/planningFunctions.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";
static const std::string GROUP_X = "arm_with_x_move";



int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE */
    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             "/home/serboba/Desktop/blenderFLEX/fetch4.urdf",  //
                                             "/home/serboba/Desktop/blenderFLEX/fetch4.srdf");


    std::string env_name = "maze3";

    auto maze_dart = darts::loadMoveItRobot(env_name,
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/" + env_name + ".urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/envs/" + env_name + ".srdf");

    auto fetch_name = fetch_dart->getName();
    auto door_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(maze_dart);
    
    create_txt_from_urdf(); 
    std::vector<Object> obj_;
    create_objects_from_txt(env_name,obj_);

    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE UNTIL HERE !!!! */
    darts::Window window(world);

    fetch_dart->setJoint("torso_lift_joint",0.25); // maze2 to avoid start collision
    fetch_dart->setJoint("move_x_axis_joint",-0.10);


    window.run([&] {

        URDF_IO input_(env_name);
        std::vector<ActionP> actions_path;
        std::vector<ActionR> actions_robot;
        getActionsFromPath(env_name, input_.group_indices, actions_path);
        translateActions(actions_path, obj_, actions_robot);

        darts::PlanBuilder builder(world);
        builder.addGroup("fetch", GROUP_X);
       // builder.setStartConfiguration({0.05, 1.32, 1.4, -0.2, 1.72, 0, 1.66, 0, 0.0,0.0,0.0}); // folded arm maybe necessary
        builder.setStartConfigurationFromWorld();
        Eigen::VectorXd start_config = builder.getStartConfiguration();
        
        
        //int chosen = 4; //maze vertical surface number
        int surface_no = 5;

        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        int i = 0;

        while (i < actions_robot.size()) {
            if (plan_to_grasp(world, window, obj_[actions_robot[i].obj_index], surface_no, true, fetch_dart,
                               start_config)) { // door1
                if(plan_to_move(world, window, obj_[actions_robot[i].obj_index], actions_robot[i], fetch_dart,
                                      maze_dart)){

                    builder.setStartConfigurationFromWorld();
                    start_config = builder.getStartConfiguration();
                    i++;
                    }
            }
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    });

    return 0;
}

