//
// Created by serboba on 06.12.21.
//


//
// Created by serboba on 12.10.21.
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
#include <algorithm>


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

std::map<std::string,std::vector<double>> d_values = {{"doorgr1",{1.56,-1.56}},{"doorgr2",{1.56,-1.56}} ,{"doorgr3",{-1.56,1.56}}};
// todo -> find new values -> sampling?
// todo after you found the solution, plan every state separately and the cube movement at the end

void reset_joints(std::shared_ptr<darts::World> &world){
    world->getRobot("maze")->setJoint("door_joint1",0.0);
    world->getRobot("maze")->setJoint("door_joint2",0.0);
    world->getRobot("maze")->setJoint("door_joint3",0.0);
    world->getRobot("maze")->setJoint("cube_joint",0.0);
    world->getRobot("maze")->setJoint("cube_joint2",0.0);
}

void print_assigned(std::map<std::string,double> assigned){
    for(auto &a : assigned){
        std::cout << "joint name : " << a.first;
        std::cout << " - joint value : " << a.second << std::endl;
    }
}


bool move_cube (std::map<std::string,double> assignment, std::shared_ptr<darts::World> world){
    darts::PlanBuilder builder(world);
    std::vector<double> goal_state_vector;

    for(auto const &a : assignment){
        builder.addGroup("maze",a.first);
        goal_state_vector.push_back(a.second);
    }

    RBX_INFO("Trying following assignment: ");
    print_assigned(assignment);

    builder.addGroup("maze", "cube_gr");
    goal_state_vector.push_back(0.0);
    goal_state_vector.push_back(0.55);


    std::vector<double> start_config(goal_state_vector.size(), 0.0);
    builder.setStartConfiguration(start_config);
    auto idk = builder.getStartConfiguration();

    builder.initialize();

    auto goal = builder.getGoalConfiguration(goal_state_vector);
    goal->setThreshold(0.01);
    builder.setGoal(goal);


    //  builder.setGoal(goal);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,false);
    rrt->setRange(0.1);
    builder.ss->setPlanner(rrt);

    builder.setup();

    ompl::base::PlannerStatus solved = builder.ss->solve(15.0);

    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        RBX_INFO("Found solution!");
        reset_joints(world);
        return true;

    }
    else{
        RBX_WARN("No solution found");
        reset_joints(world);
        return false;
    }
}

bool find_in_vector(std::vector<std::string> vec, std::string str){
    BOOST_FOREACH(std::string s, vec){
        if(s == str)
            return true;
    }
    return false;
}


bool consistent (std::map<std::string,double> assignment,std::shared_ptr<darts::World> world){
    darts::PlanBuilder builder(world);
    std::vector<double> goal_state_vector;

    RBX_INFO("Trying following assignment: ");
    print_assigned(assignment);

    for(auto const &a : assignment){
        builder.addGroup("maze",a.first);
        goal_state_vector.push_back(a.second);
    }

    std::vector<double> start_config(goal_state_vector.size(), 0.0);
    builder.setStartConfiguration(start_config);
    auto idk = builder.getStartConfiguration();
    builder.initialize();

    auto goal = builder.getGoalConfiguration(goal_state_vector);
    builder.setGoal(goal);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,false);
    rrt->setRange(0.1);
    builder.ss->setPlanner(rrt);
    builder.setup();

    ompl::base::PlannerStatus solved = builder.ss->solve(30.0);

    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {

        RBX_INFO("Found solution!");

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        reset_joints(world);

        return true;

    }
    else{
        RBX_WARN("No solution found");

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        reset_joints(world);
        return false;
    }
};


std::map<std::string,std::vector<double>> select_unassigned_joint(std::map<std::string,double> assigned_){
    std::pair<std::string,double> joints;
    std::vector<std::string> assigned_joints;
    BOOST_FOREACH(joints,assigned_){
                    assigned_joints.push_back(joints.first);
                }

    std::map<std::string , std::vector<double>>::iterator it;
    std::map<std::string,std::vector<double>> var;
    for(auto & d_var : d_values){
        if(!find_in_vector(assigned_joints,d_var.first)){
            var.insert(std::make_pair(d_var.first,d_var.second));
            return var;
        }
    }
}

void pop_element(std::map<std::string,double> &assigned_){
    for(auto it = assigned_.begin(); it != assigned_.end(); it++){
        std::cout <<"pop: " <<  it->first << std::endl;
    }
    auto it = assigned_.end();
    it--;
    assigned_.erase(it);
}

bool backtracking(std::map<std::string,double> &assigned,std::shared_ptr<darts::World> world ){
    if(move_cube(assigned,world)){
        print_assigned(assigned);
        return true;
    }

    std::map<std::string,std::vector<double>> var = select_unassigned_joint(assigned);
    if(var.size() == 0)
        return false;
    BOOST_FOREACH(double value, var.begin()->second){ // -1.57, 1.57
        std::map<std::string,double> temp_assigned = assigned;
        temp_assigned.insert(std::make_pair(var.begin()->first,value));
        if(consistent(temp_assigned,world)){
            assigned = temp_assigned;
            auto result = backtracking(assigned,world);
            if(result!=false){
                std::cout << "Found result returning: " << std::endl;
                print_assigned(assigned);
                return result;
            }
            pop_element(assigned);
        }
    }
    return false;

}

int main(int argc, char **argv)
{
    // Startup ROS
    ROS ros(argc, argv);


    auto maze_dart = darts::loadMoveItRobot("maze",
                                             "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.urdf",
                                             "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze.srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();
    world->addRobot(maze_dart);

    darts::Window window(world);




    const auto &plan_solution = [&](std::map<std::string,double> assignment) {
        darts::PlanBuilder builder(world);
        std::vector<double> goal_state_vector;

        for(auto const &a : assignment){
            builder.addGroup("maze",a.first);
            goal_state_vector.push_back(a.second);
        }
        builder.addGroup(maze_name, "cube_gr");

        RBX_INFO("Trying following assignment: ");
        print_assigned(assignment);

        goal_state_vector.push_back(0.0);
        goal_state_vector.push_back(0.55);


        std::vector<double> start_config(goal_state_vector.size(), 0.0);
        builder.setStartConfiguration(start_config);
        auto idk = builder.getStartConfiguration();

        builder.initialize();

        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));


        auto goal = builder.getGoalConfiguration(goal_state_vector);
        goal->setThreshold(0.01);
        builder.setGoal(goal);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,false);
        rrt->setRange(0.1);
        builder.ss->setPlanner(rrt);

        builder.setup();

        ompl::base::PlannerStatus solved = builder.ss->solve(120.0);

        if (solved)
        {
            RBX_INFO("Found solution!");
            while(true){
                window.animatePath(builder, builder.getSolutionPath());
            }
        }
        else
            RBX_WARN("No solution found");
    };


    window.run([&] {

        std::map<std::string,double> assigned;
        std:: cout << backtracking(assigned,world) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        plan_solution(assigned);
    });

    return 0;
}
