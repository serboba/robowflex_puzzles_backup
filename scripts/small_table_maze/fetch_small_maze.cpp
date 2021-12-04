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


using namespace robowflex;

static const std::string GROUP = "arm_with_torso";
static const std::string GROUP_X = "arm_with_x_move";



int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);

    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE */
    auto fetch_dart = darts::loadMoveItRobot("fetch",                                         //
                                             "/home/serboba/Desktop/blenderFLEX/fetch3.urdf",  //
                                             "/home/serboba/Desktop/blenderFLEX/fetch3.srdf");

    auto door_dart = darts::loadMoveItRobot("myfirst",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.urdf",
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.srdf");

    auto fetch_name = fetch_dart->getName();
    auto door_name = door_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(fetch_dart);
    world->addRobot(door_dart);

    std::string urdf_name = "maze.txt";
    std::string srdf_name = "maze_srdf.txt";

    create_objects_from_urdf(urdf_name, srdf_name);
    /* NEVER CHANGE THIS ROBOT LOADING STRUCTURE UNTIL HERE !!!! */
    darts::Window window(world);

    const auto &strech_normal = [&](Eigen::MatrixXd &normal_vec){ // add normal to the pos we calculated because wrist not positioned right
        for(int i = 0; i < 3 ;i++){
            if(normal_vec(0,i) == 0.0)
                continue;
            if(normal_vec(0,i) > 0.0)
                normal_vec(0,i) +=0.01;
            else
                normal_vec(0,i) -=0.01;
        }
        std::cout<< "SELECTED NORMAL VEC: " << normal_vec << std::endl;
    };

    const auto &get_start_state = [&](darts::PlanBuilder &builder_, std::vector<double> &config){ // to secure the start state
        Eigen::VectorXd start_config = builder_.getStartConfiguration();
        config.resize(start_config.size());
        Eigen::VectorXd::Map(&config[0], start_config.size()) = start_config;

    };

    const auto &evaluate_pos = [&](Eigen::MatrixXd &pose){
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        bool found_pose = false;
        //int max_iter = 20;
        Eigen::MatrixXd normal_v = pose.block(0, 7, 1, 3);
        Eigen::MatrixXd position = pose.block(0, 0, 1, 3);
        Eigen::MatrixXd quaternion = pose.block(0, 3, 1, 4);
        double l = 0.0;

        while(!found_pose && l<=0.30){ // HOW TO SET L VALUE?? l = stretching factor for normal
            l = l+ 0.01; // todo adjust L ? find optimal L ? how to set optimal L ?

            strech_normal(normal_v);
            Eigen::MatrixXd new_position = position + normal_v;

            std::cout <<"position calculated" << std::endl;
            std::cout << new_position << std::endl;    // pose vector
            std::cout <<"position calculated" << std::endl;

            darts::TSR::Specification start_spec;
            start_spec.setFrame(fetch_name,"wrist_roll_link","move_x_axis");
            start_spec.setPosition(new_position(0),new_position(1),new_position(2));
            start_spec.setRotation(quaternion(0),quaternion(1),quaternion(2),quaternion(3));

            auto start_tsr = std::make_shared<darts::TSR>(world,start_spec);
            start_tsr->initialize();
            start_tsr->useGroup(GROUP_X);
            start_tsr->solveWorld();

            builder.setStartConfigurationFromWorld();
            builder.initialize();

            std::vector<double> config_vec;
            get_start_state(builder,config_vec);

            darts::TSR::Specification goal_spec2;  //
            goal_spec2.setFrame(fetch_name, "wrist_roll_link", "move_x_axis");
            goal_spec2.setPoseFromWorld(world);
            auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);
            auto goal = builder.getGoalTSR(goal_tsr2);
            goal->setThreshold(0.0001);
            builder.setGoal(goal);

            auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
            builder.ss->setPlanner(rrt);
            builder.setup();

            ompl::base::ScopedState<> goal_state(builder.space);
            builder.ss->getSpaceInformation().get()->getStateSpace()->copyFromReals(goal_state.get(),config_vec);
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

            }else{
             //   OMPL_DEBUG("FALSE INVALID STATE");
            }

        }

    };

    const auto get_pose = [&](Object &obj,MatrixXd &pose_m, int &surf_no){

        MatrixXd pose_matrix = get_pose_object(obj, surf_no);
        MatrixXd temp2;
        bool guard = false;
        int temp_no = -1;
        while(!guard){ // IF WE COULD FIND A NEW POSE VALID POSE -> GUARD TRUE (POSE WRITTEN IN TEMP2)
            Eigen::Map<MatrixXd> temp(pose_matrix.data(),1,7);
            pose_matrix = get_pose_object(obj, surf_no);
            temp_no = pose_matrix(10);// last index of pose matrix is the selected surf no , pose matrix :point,quaternion,normal,surf_no
            std::cout <<"temp no: " << temp_no;
            evaluate_pos(pose_matrix); // check if we can plan to the point/reach the point
            temp2 = pose_matrix.block(0,0,1,7);
            if(temp.isApprox(temp2) == 0) // if == 0, means old pose != new pose so evaluate pos changed pose-> we found a valid pose, otherwise pose wont be changed
                guard=true;
        }
        pose_m = temp2;
        surf_no = temp_no;
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

    const auto &update_object_position = [&](Object &obj ,int axis, double value){
        obj.actual_position[axis] += value;
    };

    const auto &update_object_rotation = [&](Object &obj ,int axis, double value){ // TODO
       // std::cout << "OLD POSITION OF LINK : " << obj.actual_position << std::endl;
        obj.actual_position[axis] += value;
       // std::cout << "NEW POSITION OF LINK : " << obj.actual_position << std::endl;

    }; // TODO

    const auto &get_pos_rot_from_frame = [&](Vector3d &position,MatrixXd &rotation,Eigen::Vector3d &degrees,Object obj, int surf_no){ // rotation with euler angles sorted X Z Y NOT XYZ fml
        darts::TSR::Specification pos_spec;
        pos_spec.setFrame(fetch_name,"wrist_roll_link", "move_x_axis");
        pos_spec.setPoseFromWorld(world);
        //degrees << 0,0,0.8;


        MatrixXd new_degre = vec_to_matrix(obj.actual_rotation+ degrees);
        rotation = actual_quaternion(new_degre, surf_no);

        std::cout<< "NEW DEGREE : " << new_degre << std::endl;

        MatrixXd rotvex = get_rotated_vertex(degrees, pos_spec.getPosition(), obj.joint_xyz);
        position = matrix_to_vec(rotvex);


        std::cout << pos_spec.getPosition()[0] << "," << pos_spec.getPosition()[1] << ","<< pos_spec.getPosition()[2] << std::endl;
        std::cout << position(0) << "," << position(1) << "," << position(2) << std::endl;

        std::cout << "OLD ROTATION: " <<  std::endl;
        eigen_quaternion_to_rpy(pos_spec.getRotation());
        std::cout << "NEW ROTATION: " << std::endl;
        eigen_matrix_quaternion_to_rpy(rotation);

        // alternative !
        /*
        rotation = match_deg_to_rpy_new(vec_to_matrix(degrees), pos_spec.getRotation());


        MatrixXd rotvex = get_rotated_vertex(degrees, pos_spec.getPosition(), obj.joint_xyz);
        position = matrix_to_vec(rotvex);


        std::cout << pos_spec.getPosition()[0] << "," << pos_spec.getPosition()[1] << ","<< pos_spec.getPosition()[2] << std::endl;
        std::cout << position(0) << "," << position(1) << "," << position(2) << std::endl;

        std::cout << "OLD ROTATION: " <<  std::endl;
        eigen_quaternion_to_rpy(pos_spec.getRotation());
        std::cout << "NEW ROTATION: " << std::endl;
        eigen_matrix_quaternion_to_rpy(rotation);
    */

        //std::cout << "OLD ROTATION: " << pos_spec.getRotation().w() << "," << pos_spec.getRotation().x() <<","<<pos_spec.getRotation().y() <<","<< pos_spec.getRotation().z() << std::endl;
        //std::cout << "NEW ROTATION: " << rotation(0) <<"," << rotation(1) <<"," <<rotation(2) <<"," <<rotation(3)  << std::endl;
        /*
        tf2::Quaternion old_rot = eigen_to_tfquaternion(pos_spec.getRotation());
        tf2::Quaternion result = tf2::Quaternion(tf2::Vector3(0,0,1),0.80) * old_rot;
        rotation = tf_to_eigen_matrix_q(result);
        std::cout << "OLD ROTATION: " << pos_spec.getRotation().w() << "," << pos_spec.getRotation().x() <<","<<pos_spec.getRotation().y() <<","<< pos_spec.getRotation().z() << std::endl; std::cout << "NEW ROTATION: " << rotation(0) <<"," << rotation(1) <<"," <<rotation(2) <<"," <<rotation(3)  << std::endl;


        std::cout<< degrees<< std::endl;
        std::cout << "OLD POSITION OF POINT: "<< pos_spec.getPosition() << std::endl;
        MatrixXd rotvex = get_rotated_vertex(degrees, pos_spec.getPosition(), obj.joint_xyz);
        position = matrix_to_vec(rotvex);
        std::cout << "NEW POSITION OF POINT: "<< position << std::endl;
*/

    };

    const auto &plan_to_grasp = [&](Object &obj, int &surf_no) {
        MatrixXd pose_m(1,11);
        bool flag = false;
        // this new part secure the older pose from the earlier motion
        darts::PlanBuilder builder2(world);
        builder2.addGroup(fetch_name, GROUP_X);
        builder2.setStartConfigurationFromWorld();
        auto start_config = builder2.getStartConfiguration();
        if(start_config[0] == 0.0) // if its the starting(beginning) motion the arm may be collide therefore adjust the torso as you want
            start_config[0] += 0.10;


        while(!flag){
            get_pose(obj, pose_m, surf_no);
            
            /* TODO SECURE STARTING POSE FROM EARLIER ROTATION  // BUILDER BEFORE PLAN TO GRASP -> INF LOOP? OR SET JOINT TO 0 ?*/
            darts::PlanBuilder builder(world);
            builder.addGroup(fetch_name, GROUP_X);
            builder.setStartConfiguration(start_config);
            builder.initialize();

            ompl::base::PlannerStatus solved;
            darts::TSR::Specification goal_spec;
            goal_spec.setFrame(fetch_name, "wrist_roll_link", "move_x_axis");
            goal_spec.setPose(pose_m); // HOP

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

            if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION){
                RBX_INFO("Found solution!");
                window.animatePath(builder, builder.getSolutionPath());
                flag = true;
            }else{
                RBX_WARN("No solution found");
            }
        }
    };

    const auto &plan_to_rotate_rpy = [&](Object &obj, bool &flag, Eigen::Vector3d &degrees, int surf_no) { // TODO TAKE DEGREES AS INPUT
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        builder.addGroup(door_name, obj.group_name);
        builder.setStartConfigurationFromWorld();
        auto idk = builder.getStartConfiguration();

        Eigen::MatrixXd rotation;
        Eigen::Vector3d position;
        //std::cout <<   "SURFF NO :" <<   surf_no << std::endl;
        get_pos_rot_from_frame(position,rotation,degrees,obj, surf_no);

        darts::TSR::Specification con_spec;
        con_spec.setBase(door_name, obj.link_name);
        con_spec.setTarget(fetch_name,"wrist_roll_link");
        con_spec.setPoseFromWorld(world);

        auto cons_tsr = std::make_shared<darts::TSR>(world,con_spec);
        builder.addConstraint(cons_tsr);
        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name,"wrist_roll_link","move_x_axis");
        goal_spec.setPosition(position);
        goal_spec.setRotation(rotation(0),rotation(1),rotation(2),rotation(3));

        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        goal->setThreshold(0.01); // maybe find better threshold?
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        //  rrt->setRange(0.10);
        builder.ss->setPlanner(rrt);
        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(30.0);
        goal->stopSampling();

        OMPL_DEBUG("HAVE EXACT SOLUTION PATH: ");
        std::cout << builder.ss->haveExactSolutionPath() << std::endl;

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION )
        {
            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            flag = true;
            //update_object_position(obj,axis,value);
        }
        else{
            RBX_WARN("No solution found");
            door_dart->setJoint(obj.joint_name,0.0);
        }
    };

    const auto &plan_to_move_xyz_axis = [&](Object &obj, bool &flag, int axis, double value) {
        darts::PlanBuilder builder(world);
        builder.addGroup(fetch_name, GROUP_X);
        builder.addGroup(door_name, obj.group_name);
        builder.setStartConfigurationFromWorld();
        auto idk = builder.getStartConfiguration();

        darts::TSR::Specification pos_spec;
        pos_spec.setFrame(fetch_name,"wrist_roll_link", "move_x_axis");
        pos_spec.setPoseFromWorld(world);

        auto rotation = pos_spec.getRotation();
        Eigen::Vector3d position = pos_spec.getPosition();
        std::cout << "OLD POSITION : " << position << std::endl;

        darts::TSR::Specification con_spec;
        con_spec.setBase(door_name, obj.link_name);
        con_spec.setTarget(fetch_name,"wrist_roll_link");
        con_spec.setPoseFromWorld(world);
        auto cons_tsr = std::make_shared<darts::TSR>(world,con_spec);
        builder.addConstraint(cons_tsr);

        builder.initialize();

        darts::TSR::Specification goal_spec;
        goal_spec.setFrame(fetch_name,"wrist_roll_link","move_x_axis");
        handle_axis(axis,value,position);
        goal_spec.setPosition(position);

        std::cout << "NEW POSITION : " << position << std::endl;
        goal_spec.setRotation(rotation);
        goal_spec.print(std::cout);
        auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
        auto goal = builder.getGoalTSR(goal_tsr);
        goal->setThreshold(0.01); // maybe find better threshold?
        builder.setGoal(goal);

        auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
        //  rrt->setRange(0.10);
        builder.ss->setPlanner(rrt);
        builder.setup();

        goal->startSampling();
        ompl::base::PlannerStatus solved = builder.ss->solve(600000.0);
        goal->stopSampling();

        OMPL_DEBUG("HAVE EXACT SOLUTION PATH: ");
        std::cout << builder.ss->haveExactSolutionPath() << std::endl;

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION )
        {

            RBX_INFO("Found solution!");
            window.animatePath(builder, builder.getSolutionPath());
            flag = true;
            update_object_position(obj,axis,value);
        }
        else{
            RBX_WARN("No solution found");
            door_dart->setJoint(obj.joint_name,0.0);
        }
    };



    window.run([&] {
         create_txt_from_urdf();
        std::vector<Object> obj_s = get_objects();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        bool flag = false;
        int chosen_surface_no = 5;

        std::vector<Eigen::Vector3d> degrees;
        Eigen::Vector3d  d_;
        d_ << 0.0,0.0,-1.50;
        degrees.push_back(d_);
        d_ << 0.0,0.0,-1.50;
        degrees.push_back(d_);
        d_ << 0.0,0.0,1.50;
        degrees.push_back(d_);

        for(int i =1; i< obj_s.size()+1 ; i ++){ // todo adjust degrees
            if(i == 4){
                plan_to_grasp(obj_s[0],chosen_surface_no); // door1
                plan_to_move_xyz_axis(obj_s[0],flag,1,0.50);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

            }

            while(!flag){
                plan_to_grasp(obj_s[i],chosen_surface_no); // door1
                //plan_to_move_xyz_axis(obj_s[0],flag,0,0.2);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                plan_to_rotate_rpy(obj_s[i],flag,degrees[i-1], chosen_surface_no);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            flag=false;
            }


    });
// todo integrate joint type from input f.e. if obj.j_type = "revolute" --> match_deg_to_py

    return 0;
}
