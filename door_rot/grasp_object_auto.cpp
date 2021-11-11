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


#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/Constraint.h>
#include <ompl/geometric/SimpleSetup.h>

#include <moveit_msgs/MotionPlanRequest.h>

#include <robowflex_library/class_forward.h>


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


/*
for ( auto it = VectorOfPairs.begin(); it != VectorOfPairs.end(); it++ )
{
// To get hold of the class pointers:
auto pClass1 = it->first;
auto pClass2 = it->second;
}*/


int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE */
    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             "/home/serboba/Desktop/blenderFLEX/fetch3.urdf",  //
                                             "/home/serboba/Desktop/blenderFLEX/fetch3.srdf");

    auto door_dart = darts::loadMoveItRobot("myfirst",
                                            "/home/serboba/Desktop/blenderFLEX/cube_scene.urdf",
                                            "/home/serboba/Desktop/blenderFLEX/cube_scene.srdf");

    auto fetch_name = fetch_dart->getName();
    auto door_name = door_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(door_dart);




    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE UNTIL HERE !!!! */
    darts::Window window(world);


    const auto &evaluate_pos = [&](Eigen::MatrixXd &pose){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        bool found_pose = false;
        //int max_iter = 20;
        Eigen::MatrixXd pose_m = pose.block(0, 0, 1, 7);
        Eigen::MatrixXd normal_v = pose.block(0, 7, 1, 3);
        Eigen::MatrixXd position = pose_m.block(0, 0, 1, 3);
        Eigen::MatrixXd quaternion = pose_m.block(0, 3, 1, 4);
        double l = 0.0;

        while(!found_pose && l<=0.30){ // HOW TO SET L VALUE??
            l = l+ 0.01;
        //    std::cout << "L RATIO" << l << std::endl; // adjust normals by l
            for(int i = 0; i < 3 ;i++){
                if(normal_v(0,i) == 0.0)
                    continue;
                if(normal_v(0,i) > 0.0)
                    normal_v(0,i) +=0.01;
                else
                    normal_v(0,i) -=0.01;
            }
            Eigen::MatrixXd new_position = position + normal_v;

            std::cout <<"position calculated" << std::endl;
            std::cout << new_position << std::endl;    // pose vector
            std::cout <<"position calculated" << std::endl;

            darts::TSR::Specification start_spec;
            start_spec.setFrame(fetch_name,"wrist_roll_link","moving_link");
            start_spec.setPosition(new_position(0,0),new_position(0,1),new_position(0,2));
            start_spec.setRotation(quaternion(0,0),quaternion(0,1),quaternion(0,2),quaternion(0,3));
            auto start_tsr = std::make_shared<darts::TSR>(world,start_spec);
            start_tsr->initialize();
            start_tsr->useGroup(GROUP_X);
            start_tsr->solveWorld();

            builder.setStartConfigurationFromWorld();
            builder.initialize();

            Eigen::VectorXd config = builder.getStartConfiguration();
            std::vector<double> v2;
            v2.resize(config.size());
            Eigen::VectorXd::Map(&v2[0], config.size()) = config;

            ompl::base::PlannerStatus solved;

            darts::TSR::Specification goal_spec2;  //
            goal_spec2.setFrame(fetch_name, "wrist_roll_link", "moving_link");
            goal_spec2.setPoseFromWorld(world);
            auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);
            auto goal = builder.getGoalTSR(goal_tsr2);
            goal->setThreshold(0.0001);
            builder.setGoal(goal);

            auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
            builder.ss->setPlanner(rrt);
            builder.setup();

            ompl::base::ScopedState<> goal_state(builder.space);
            builder.ss->getSpaceInformation().get()->getStateSpace()->copyFromReals(goal_state.get(),v2);

        //    std::cout << "SATISFIES BOUNDS ?? " << builder.info->satisfiesBounds(goal_state.get()) << std::endl;
        //    std::cout << "IS VALID?S ?? " << builder.info->isValid(goal_state.get()) << std::endl;

        //    if (builder.info->satisfiesBounds(goal_state.get()) && builder.info->isValid(goal_state.get()))
            if ( builder.info->isValid(goal_state.get()))
            {
                found_pose=true;
                OMPL_DEBUG("TRUE STATE VALID");
                Eigen::MatrixXd result(1,7);
                result << new_position,quaternion;
                pose = result;
                OMPL_DEBUG("NEW POSE VECTOR");
                std::cout << pose << std::endl;
                //builder.setStartConfiguration(config);
            }else{
                OMPL_DEBUG("FALSE INVALID STATE");
                //builder.setStartConfiguration(config);
            }

        }

    };


    const auto get_pose = [&](MatrixXd &pose_m, int object_no){
        std::vector<std::pair<std::string,Eigen::MatrixXd>> pose_object = get_pose_object(object_no);
        auto joint_group_name = pose_object.begin()->first;
        MatrixXd pose_matrix = pose_object.begin()->second;

        MatrixXd temp2;
        bool guard = false;
        while(!guard){ // IF WE COULD FIND A NEW POSE VALID POSE -> GUARD TRUE (POSE WRITTEN IN TEMP2)
            Eigen::Map<MatrixXd> temp(pose_matrix.data(),1,7);
            //std::cout << "POSE : " << pose_matrix << std::endl;
            pose_matrix = get_pose_object(object_no).begin()->second;
            evaluate_pos(pose_matrix);
            temp2 = pose_matrix.block(0,0,1,7);
            if(temp.isApprox(temp2) == 0) // IF EQUAL WE DIDNT FIND VALID POSE
                guard=true;
        }
        pose_m = temp2;
    };


    const auto &plan_to_grasp = [&](int object_no) {
        MatrixXd pose_m(1,10);
        bool flag = false;
        int counter = 0; // count tries
        while(!flag){
            get_pose(pose_m, object_no);
            counter = 0;
            while(pose_m.cols()>8 && counter <=3){
                get_pose(pose_m, object_no);
                counter++;
            }

            /* TODO SECURE STARTING POSE FROM EARLIER ROTATION */
            darts::PlanBuilder builder(world);
            builder.addGroup(fetch_name, GROUP_X);
            builder.setStartConfigurationFromWorld();
            builder.initialize();

            ompl::base::PlannerStatus solved;
            darts::TSR::Specification goal_spec;
            goal_spec.setFrame(fetch_name, "wrist_roll_link", "moving_link");

            goal_spec.setPose(pose_m);
            //goal_spec.print(std::cout);
            auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
            auto goal = builder.getGoalTSR(goal_tsr);

            goal->setThreshold(0.0001);
            builder.setGoal(goal);

            auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
            //rrt->setRange(0.01);
            builder.ss->setPlanner(rrt);
            builder.setup();

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            goal->startSampling();
            solved = builder.ss->solve(10.0);
            goal->stopSampling();

            if (solved){
                RBX_INFO("Found solution!");
                window.animatePath(builder, builder.getSolutionPath());
                flag = true;
            }else{
                RBX_WARN("No solution found");
            }
        }
    };

    const auto &handle_axis = [&](int axis, double value,Eigen::Vector3d &pos){
        if (axis ==0)
            pos.x()+= value;
        else if(axis==1)
            pos.y()+= value;
        else if(axis==2)
            pos.z() += value;
        else
            OMPL_INFORM("INVALID AXIS");
    };

    const auto &plan_to_move_xyz_axis = [&](std::string group_name, std::string link_name, std::string joint_name, bool &flag, int axis, double value) {
        darts::PlanBuilder builder(world);
        robowflex::darts::WorldPtr world_temp = builder.world;
        builder.addGroup(fetch_name, GROUP_X);
        builder.addGroup(door_name, group_name);
        builder.setStartConfigurationFromWorld();
        auto idk = builder.getStartConfiguration();

        darts::TSR::Specification pos_spec;
        pos_spec.setFrame(fetch_name,"wrist_roll_link", "moving_link");
        pos_spec.setPoseFromWorld(world);

        auto rotation = pos_spec.getRotation();
        Eigen::Vector3d position = pos_spec.getPosition();


        darts::TSR::Specification con_spec;
        con_spec.setBase(door_name, link_name);
        con_spec.setTarget(fetch_name,"wrist_roll_link");
        con_spec.setPoseFromWorld(world);

        auto cons_tsr = std::make_shared<darts::TSR>(world,con_spec);
        builder.addConstraint(cons_tsr);

        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name,"wrist_roll_link","moving_link");
        handle_axis(axis,value,position);
        goal_spec.setPosition(position);
        goal_spec.setRotation(rotation);

        //std::cout<< "GOAL" <<    std::endl;
        //goal_spec.print(std::cout);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        goal->setThreshold(0.01); // maybe find better threshold?
        builder.setGoal(goal);

        //std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        //  rrt->setRange(0.10);
        builder.ss->setPlanner(rrt);
        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(60.0);
        goal->stopSampling();

        OMPL_DEBUG("HAVE EXACT SOLUTION PATH: ");
        std::cout << builder.ss->haveExactSolutionPath() << std::endl;

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION )
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            flag = true;
        }
        else{
            RBX_WARN("No solution found");
            door_dart->setJoint(joint_name,0.0);
        }
    };


    window.run([&] {
       // create_txt_from_urdf();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        bool flag = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        double threshold_multiplier = 1.0;

        while(!flag){
            plan_to_grasp(2); // door1
            plan_to_move_xyz_axis("doorgr1","door1", "door_joint1",flag,0,0.2);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        flag=false;

        while(!flag) {
            plan_to_grasp(3); // door2
            plan_to_move_xyz_axis("doorgr2", "door2","door_joint2", flag, 0, 0.2);
        }
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        flag = false;
        while(!flag) {
            plan_to_grasp(1); // cube
            plan_to_move_xyz_axis("cube_gr", "cube1","cube_joint", flag, 1, -0.75);
       }

        });

    return 0;
}



