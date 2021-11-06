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

    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             "/home/serboba/Desktop/blenderFLEX/fetch2.urdf",  //
                                             "/home/serboba/Desktop/blenderFLEX/fetch2.srdf");
    auto fetch_name = fetch_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);

    auto door_dart = darts::loadMoveItRobot("objects3",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/doors.urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/doors.srdf");
    world->addRobot(door_dart);
    darts::Window window(world);


    const auto &evaluate_pos = [&](Eigen::MatrixXd &pose){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);

        bool found_pose = false;

        Eigen::MatrixXd pose_m = pose.block(0, 0, 1, 7);
        Eigen::MatrixXd normal_v = pose.block(0, 7, 1, 3);
        std::cout << "nodoor pos" << std::endl;
        std::cout << pose << std::endl;
        std::cout << normal_v << std::endl;

        int max_iter = 20;
        Eigen::MatrixXd position = pose_m.block(0, 0, 1, 3);
        Eigen::MatrixXd quaternion = pose_m.block(0, 3, 1, 4);
        double l = 0.0;
        //pose_matrix.block(i, j, 1, 3)

        while(!found_pose && l<=0.20){
            l = l+ 0.01;
            std::cout << "YOOO  L RATIO " << l << std::endl;
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

            auto config = builder.getStartConfiguration();

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

            if (builder.info->satisfiesBounds(goal_state.get()) && builder.info->isValid(goal_state.get()))
            {
                found_pose=true;
                OMPL_DEBUG("TRUEEE POSITION COMING");
                std::cout << new_position << std::endl;
                Eigen::MatrixXd result(1,7);
                result << new_position,quaternion;
                pose = result;
                OMPL_DEBUG("NEW POSE VECTOR");
                std::cout << pose << std::endl;
            }else{
                OMPL_DEBUG("FALSSSSSSSSSSEEEEEE");
            }

        }

    };

    const auto &plan_to_grasp = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        builder.setStartConfigurationFromWorld();
        builder.initialize();

        ompl::base::PlannerStatus solved;

        darts::TSR::Specification goal_spec2;  // IF GOAL NOT AS CONFIG
        goal_spec2.setFrame(fetch_name, "wrist_roll_link", "moving_link");
        std::vector<std::pair<std::string,Eigen::MatrixXd>> get_m = get_pose_object(1);
        auto gr_name = get_m.begin()->first;
        auto pose_mm = get_m.begin()->second;

        std::cout << "POSE MM BEFORE : " << pose_mm << std::endl;
        bool guard = false;
        while(!guard){
            auto temp = pose_mm.block(0,0,1,7);
            get_m = get_pose_object(1);
            pose_mm = get_m.begin()->second;
            evaluate_pos(pose_mm);

            std::cout << "POSE MMM : " << pose_mm << std::endl;
            std::cout << "TEMP : " << temp << std::endl;
            auto temp2 = pose_mm.block(0,0,1,7);

            std::cout << temp.isApprox(temp2) << std::endl;
            if(temp.isApprox(temp2) == 0)
                guard=true;
            OMPL_DEBUG("STOPPPPPP");
        }
       // builder.setStartConfiguration(temp_wr);
        std::cout << "POSE MMM BEFORE SEEEEEEEEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTT : " << pose_mm << std::endl;
        goal_spec2.setPose(pose_mm);
        auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);
        auto goal = builder.getGoalTSR(goal_tsr2);
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
        }else{
            RBX_WARN("No solution found");
        }
    };

    const auto &plan_to_pick = [&]() {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        builder.addGroup(door_dart->getName(), "doorgr1");



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
        con_spec.setBase(door_dart->getName(),"door1");
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
        bool tes1 = goal_tsr2->solveWorld();
        std::cout << "TESSSSST 21111111" << std::endl;
        std::cout << tes1 << std::endl;

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
   //     plan_to_pick();
    });

    return 0;
}


