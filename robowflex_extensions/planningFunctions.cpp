//
// Created by serboba on 05.01.22.
//

#include <robowflex_dart/planningFunctions.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
using namespace robowflex;

static const std::string GROUP_X = "arm_with_x_move";



bool plan_to_fold_arm(std::shared_ptr<darts::World> &world, darts::Window &window ) { // new position =  world position
    darts::PlanBuilder builder(world);
    builder.addGroup("TODO", GROUP_X);
    builder.setStartConfigurationFromWorld();
    auto start_config = builder.getStartConfiguration();
    builder.initialize();

    auto goal = builder.getGoalConfiguration({0.05, 1.32, 1.4, -0.2, 1.72, 0, 1.66, 0, start_config[8],start_config[9],start_config[10] });
    builder.setGoal(goal);
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
    builder.ss->setPlanner(rrt);
    builder.setup();
    ompl::base::PlannerStatus solved = builder.ss->solve(20.0);


    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION )
    {
        RBX_INFO("Found solution!");
        window.animatePath(builder, builder.getSolutionPath());
        return true;
    }
    else{
        RBX_WARN("No solution found");
        return false;
    }
};


void handle_axis (int axis, double value,Eigen::Vector3d &pos){
    if (axis ==0)
        pos.x()+= value;
    else if(axis==1)
        pos.y()+= value;
    else if(axis==2)
        pos.z() += value;
    else
        OMPL_INFORM("INVALID AXIS");
};

void strech_normal(Eigen::MatrixXd &normal_vec){ // add normal to the pos we calculated because wrist not positioned right
    for(int i = 0; i < 3 ;i++){ // TODO NORMAL DOWNWARDS (NOW UPWARDS) IF BOTH CORRECT OR ONE CORRECT ITS THE RIGHT POSITION BUT FIRST GO UP
        if(normal_vec(0,i) == 0.0)
            continue;
        if(normal_vec(0,i) > 0.0)
            normal_vec(0,i) +=0.01;
        else
            normal_vec(0,i) -=0.01;
    }
    std::cout<< "SELECTED NORMAL VEC: " << normal_vec << std::endl;

}

void get_start_state (darts::PlanBuilder &builder_, std::vector<double> &config){ // to secure the start state
    Eigen::VectorXd start_config = builder_.getStartConfiguration();
    config.resize(start_config.size());
    Eigen::VectorXd::Map(&config[0], start_config.size()) = start_config;

}

void set_start_grasp_pos(darts::PlanBuilder &builder_,std::shared_ptr<darts::World> &world,
                         Eigen::MatrixXd new_position,Eigen::MatrixXd quaternion){
    std::cout <<"position calculated" << std::endl;
    std::cout << new_position << std::endl;    // pose vector
    std::cout <<"position calculated" << std::endl;

    darts::TSR::Specification start_spec;
    start_spec.setFrame("fetch","wrist_roll_link","move_x_axis");
    start_spec.setPosition(new_position(0),new_position(1),new_position(2));
    start_spec.setRotation(quaternion(0),quaternion(1),quaternion(2),quaternion(3));

    auto start_tsr = std::make_shared<darts::TSR>(world,start_spec);
    start_tsr->initialize();
    start_tsr->useGroup(GROUP_X);
    start_tsr->solveWorld();

    builder_.setStartConfigurationFromWorld();
    builder_.initialize();
}

void set_goal_from_world(darts::PlanBuilder &builder_,std::shared_ptr<darts::World> &world){
    darts::TSR::Specification goal_spec2;  //
    goal_spec2.setFrame("fetch", "wrist_roll_link", "move_x_axis");
    goal_spec2.setPoseFromWorld(world);
    auto goal_tsr2 = std::make_shared<darts::TSR>(world, goal_spec2);
    auto goal = builder_.getGoalTSR(goal_tsr2);
    goal->setThreshold(0.0001);
    builder_.setGoal(goal);
}

void evaluate_pos (std::shared_ptr<darts::World> world,Eigen::MatrixXd &pose, bool normals_help){
    darts::PlanBuilder builder(world);
    builder.addGroup("fetch", GROUP_X);
    bool found_pose = false;
    //int max_iter = 20;
    Eigen::MatrixXd normal_v = pose.block(0, 7, 1, 3);
    Eigen::MatrixXd position = pose.block(0, 0, 1, 3);
    Eigen::MatrixXd quaternion = pose.block(0, 3, 1, 4);
    double l = 0.0;
    normals_help = true; // TODO SET AS PARAMETER IN SCRIPTS
    while((!found_pose && l<=0.30)){ // HOW TO SET L VALUE?? l = stretching factor for normal // new -> disable/enable strech normal depending on what obj you grasp
        l = l+ 0.01; // todo adjust L ? find optimal L ? how to set optimal L ?

        if(normals_help)
            strech_normal(normal_v);


        std::vector<double> start_vec;
        get_start_state(builder,start_vec);

        ompl::base::State * start_state = builder.rspace->allocState();
        builder.rspace->copyFromReals(start_state,start_vec);

        Eigen::MatrixXd new_position = position + normal_v;
        set_start_grasp_pos(builder,world,new_position,quaternion);

        std::vector<double> config_vec;
        get_start_state(builder,config_vec);
        set_goal_from_world(builder,world);

        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
        builder.ss->setPlanner(rrt);
        builder.setup();

        // ompl::base::ScopedState<> goal_state(builder.space);
        ompl::base::State * goal_state = builder.rspace->allocState();
        builder.rspace->copyFromReals(goal_state,config_vec);
        // builder.ss->getSpaceInformation().get()->getStateSpace()->copyFromReals(goal_state.get(),config_vec);
        //    std::cout << "SATISFIES BOUNDS ?? " << builder.info->satisfiesBounds(goal_state.get()) << std::endl;
        std::cout << "IS VALID?S ?? " << builder.rinfo->isValid(goal_state) << std::endl;
        std::cout << "CHECKMOT?S ?? " << builder.rinfo->checkMotion(start_state,goal_state) << std::endl;
        //    if (builder.info->satisfiesBounds(goal_state.get()) && builder.info->isValid(goal_state.get()))
        if ( builder.rinfo->isValid(goal_state)){
            //   if ( builder.rinfo->checkMotion(start_state,goal_state)){
            found_pose=true;
            OMPL_DEBUG("TRUE STATE VALID");
            Eigen::MatrixXd result(1,7);
            result << new_position,quaternion;
            pose = result;
            OMPL_DEBUG("NEW POSE VECTOR");
            std::cout << pose << std::endl;
            builder.rspace->freeState(goal_state);
            builder.rspace->freeState(start_state);
        }else{
            if(!normals_help)
                return;
            //   OMPL_DEBUG("FALSE INVALID STATE");
        }
    }
}


Eigen::MatrixXd get_pose(std::shared_ptr<darts::World> world, Object &obj, int &surf_no, bool normals_help){

    MatrixXd pose_matrix = get_pose_object(obj, surf_no);
    MatrixXd temp2;
    bool guard = false;
    int temp_no = -1;
    while(!guard){                                                  // IF WE COULD FIND A NEW POSE VALID POSE -> GUARD TRUE (POSE WRITTEN IN TEMP2)
        Eigen::Map<MatrixXd> temp(pose_matrix.data(),1,7);
        pose_matrix = get_pose_object(obj, surf_no);
        temp_no = pose_matrix(10);                             // last index of pose matrix is the selected surf no , pose matrix :point,quaternion,normal,surf_no
        evaluate_pos(world,pose_matrix, normals_help);            // check if we can plan to the point/reach the point
        temp2 = pose_matrix.block(0,0,1,7);
        if(temp.isApprox(temp2) == 0)                          // if == 0, means old pose != new pose so evaluate pos changed pose-> we found a valid pose, otherwise pose wont be changed
            guard=true;
    }
    surf_no = temp_no;

    return temp2;
}


bool grasp(std::shared_ptr<darts::World> &world, darts::Window &window, Object &obj, int &surf_no,
           bool normals_help, std::shared_ptr<darts::Robot> &robot_, Eigen::VectorXd old_config){

    Eigen::MatrixXd pose_m = get_pose(world,obj,  surf_no, normals_help);

    darts::PlanBuilder builder(world);
    builder.addGroup(robot_->getName(), GROUP_X);
    std::cout << "old config : " << std::endl;
    std::cout << old_config << std::endl;

    builder.setStartConfiguration(old_config);
    builder.initialize();

    ompl::base::PlannerStatus solved;
    darts::TSR::Specification goal_spec;
    goal_spec.setFrame(robot_->getName(), "wrist_roll_link", "move_x_axis");
    goal_spec.setPose(pose_m); // HOP

    std::cout << "pose : " << pose_m << std::endl;
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
    solved = builder.ss->solve(5.0);
    goal->stopSampling();


    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION){
        RBX_INFO("Found solution!");
        window.animatePath(builder, builder.getSolutionPath());
        return true;
    }else{
        RBX_WARN("No solution found");
        return false;
    }
}

void add_constraint_to_builder(std::shared_ptr<darts::World> &world, darts::PlanBuilder &builder, Object &obj,
                               std::shared_ptr<darts::Robot> &robot_, std::shared_ptr<darts::Robot> &obj_robot_){

    darts::TSR::Specification con_spec;
    con_spec.setBase(obj_robot_->getName(), obj.link.name);
    con_spec.setTarget(robot_->getName(),"wrist_roll_link");
    con_spec.setPoseFromWorld(world);

    auto cons_tsr = std::make_shared<darts::TSR>(world,con_spec);
    builder.addConstraint(cons_tsr);
}

std::shared_ptr<darts::TSR> create_goalTSR(Eigen::MatrixXd rotation, Eigen::Vector3d position, std::string robot_name,
                                           std::shared_ptr<darts::World> &world){
    darts::TSR::Specification goal_spec;
    goal_spec.setFrame(robot_name,"wrist_roll_link","move_x_axis");
    std::cout << "pos in cre : " << position << std::endl;
    goal_spec.setPosition(position);
    goal_spec.setRotation(rotation(0),rotation(1),rotation(2),rotation(3));

    auto goal_tsr = std::make_shared<darts::TSR>(world, goal_spec);
    return goal_tsr;
}

bool plan_to_grasp(std::shared_ptr<darts::World> &world, darts::Window &window, Object &obj, int &surf_no, bool normals_help,
                   std::shared_ptr<darts::Robot> &robot_,Eigen::VectorXd start_config) {
    MatrixXd pose_m(1,11);
    // this new part secure the older pose from the earlier motion

    // builder.setStartConfiguration(start_config);

    std::cout << "START:  " << std::endl;
    std::cout << start_config << std::endl;

    while(!grasp(world, window, obj, surf_no,  normals_help, robot_,start_config)){
        std::cout << "start_config::::::::::::: " << std::endl;
        std::cout << start_config << std::endl;
        //set start config
    }
    return true;
}

void get_pos_rot_from_frame (std::shared_ptr<darts::World> &world, Vector3d &position,MatrixXd &rotation,Eigen::Vector3d &degrees,Object obj, int surf_no,
                             std::string robot_name){ // rotation with euler angles sorted X Z Y NOT XYZ fml
    darts::TSR::Specification pos_spec;
    pos_spec.setFrame(robot_name,"wrist_roll_link", "move_x_axis");
    pos_spec.setPoseFromWorld(world);

    MatrixXd new_degre = vec_to_matrix(obj.actual_rotation+ degrees);
    rotation = actual_quaternion(new_degre, surf_no);

    std::cout<< "NEW DEGREE : " << new_degre << std::endl;
    MatrixXd rotvex = get_rotated_vertex(degrees, pos_spec.getPosition(), (obj.joints.pos+obj.link.pos));
    position = matrix_to_vec(rotvex);

}



bool plan_to_rotate_rpy (std::shared_ptr<darts::World> &world, darts::Window &window,Object &obj, Eigen::Vector3d &degrees, int surf_no,
                         std::shared_ptr<darts::Robot> &robot_, std::shared_ptr<darts::Robot> &obj_robot_) {
    darts::PlanBuilder builder(world);
    builder.addGroup(robot_->getName(), GROUP_X);
    builder.addGroup(obj_robot_->getName(), obj.group_name);
    builder.setStartConfigurationFromWorld();
    auto idk = builder.getStartConfiguration();
    std::cout << idk << std::endl;

    Eigen::MatrixXd rotation;
    Eigen::Vector3d position;
    //std::cout <<   "SURFF NO :" <<   surf_no << std::endl;
    get_pos_rot_from_frame(world,position,rotation,degrees,obj, surf_no, robot_->getName());

    add_constraint_to_builder(world,builder,obj,robot_,obj_robot_);

    builder.initialize();

    auto goal = builder.getGoalTSR(create_goalTSR(rotation,position,robot_->getName(),world));
    goal->setThreshold(0.01); // maybe find better threshold?
    builder.setGoal(goal);

    auto rrt = std::make_shared<ompl::geometric::FMT>(builder.info);
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
        return true;
        //update_object_position(obj,axis,value);
    }
    else{
        RBX_WARN("No solution found");
        Eigen::VectorXd reset(1);
        reset << 0;
        std::cout << "OBJ RRR : " << obj_robot_->getName() << std::endl;
        std::cout << "OBJ RRR : " << obj.group_name << std::endl;
        world->getRobot(obj_robot_->getName())->setGroupState(obj.group_name,reset); // TODO
        return false;
    }
}

void translateActionToGoalRegion(Vector3d &position,Eigen::Quaterniond &rotation,ActionR action_, Object &obj)
{
    int surfno = 5;
    std::cout << rotation.coeffs() << std::endl;
    //action_.rpy(1) = 1.5708;
    if(action_.pos.isZero()){
        std::cout<<"YES POS ZERO, probably door"<<std::endl;
        std::cout<< "old pos : " << position << std::endl;
        rotation = matrix_to_quaternion(actual_quaternion(vec_to_matrix(action_.rpy+obj.actual_rotation), surfno));

        Vector3d sg = action_.rpy;
        position = matrix_to_vec(get_rotated_vertex(sg, position, obj.joints.pos));
        std::cout<< "new pos : " << position << std::endl;
    }
    else
    {

        std::cout<< "old pos : " << position << std::endl;
        position += action_.pos;
        std::cout<< "new pos : " << position << std::endl;

    }

}


bool plan_to_move (std::shared_ptr<darts::World> &world,darts::Window &window,Object &obj, ActionR action_,
                            std::shared_ptr<darts::Robot> &robot_, std::shared_ptr<darts::Robot> &obj_robot_ ) {
    darts::PlanBuilder builder(world);
    builder.addGroup(robot_->getName(), GROUP_X);
    builder.addGroup(obj_robot_->getName(), obj.group_name);
    builder.setStartConfigurationFromWorld();
    auto idk = builder.getStartConfiguration();
    std::cout << int(world->getRobot(obj_robot_->getName())->getGroupJoints(obj.group_name).size()) << std::endl;
    Eigen::VectorXd backup_state(int(world->getRobot(obj_robot_->getName())->getGroupJoints(obj.group_name).size()));
    world->getRobot(obj_robot_->getName())->getGroupState(obj.group_name,backup_state);

    darts::TSR::Specification pos_spec;
    pos_spec.setFrame(robot_->getName(),"wrist_roll_link", "move_x_axis");
    pos_spec.setPoseFromWorld(world);
    //pos_spec.print(std::cout);
    auto rotation = pos_spec.getRotation();
    Eigen::Vector3d position = pos_spec.getPosition();
    std::cout << "OLD POSITION : " << position << std::endl;

    add_constraint_to_builder(world,builder,obj,robot_,obj_robot_);

    builder.initialize();

    translateActionToGoalRegion(position,rotation,action_,obj);

    auto goal = builder.getGoalTSR(create_goalTSR(eigen_quaternion_to_matrix(rotation),position,robot_->getName(),world));
    //goal->getTSRSet()->print(std::cout);
    goal->setThreshold(0.001);
    builder.setGoal(goal);

    auto rrt = std::make_shared<ompl::geometric::KPIECE1>(builder.info);
    builder.ss->setPlanner(rrt);
    builder.setup();

    goal->startSampling();
    ompl::base::PlannerStatus solved = builder.ss->solve(180.0);
    goal->stopSampling();

    OMPL_DEBUG("HAVE EXACT SOLUTION PATH: ");
    std::cout << builder.ss->haveExactSolutionPath() << std::endl;
    builder.ss->simplifySolution();

    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION )
    {

        RBX_INFO("Found solution!");
        window.animatePath(builder, builder.getSolutionPath(true));
        obj.actual_position += action_.pos;
        obj.actual_rotation += action_.rpy;

        return true;
    }
    else{
        RBX_WARN("No solution found");
        world->getRobot(obj_robot_->getName())->setGroupState(obj.group_name,backup_state); // TODO
        return false;
    }
}

bool plan_to_move_robot (std::shared_ptr<darts::World> &world,darts::Window &window,Vector3d new_position) {
    darts::PlanBuilder builder(world);
    builder.addGroup("fetch", GROUP_X);
    builder.setStartConfigurationFromWorld();
    auto start_config = builder.getStartConfiguration();
    builder.initialize();

    std::vector<double> goal_config = {0.05, 1.32, 1.4, -0.2, 1.72, 0, 1.66, 0, new_position[0],new_position[1],0.0 }; // y axis first, x axis second
    auto goal = builder.getGoalConfiguration(goal_config);
    builder.setGoal(goal);
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
    builder.ss->setPlanner(rrt);
    builder.setup();
    ompl::base::PlannerStatus solved = builder.ss->solve(20.0);


    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION )
    {
        RBX_INFO("Found solution!");
        while(true)
            window.animatePath(builder, builder.getSolutionPath());

    }
    else{
        RBX_WARN("No solution found");
        return false;
    }
}
