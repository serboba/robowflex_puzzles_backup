//
// Created by serboba on 15.12.21.
//

//
// Created by serboba on 11.12.21.
//

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


std::map<std::string,std::vector<double>> d_values = {{"doorgr1",{1.56,-1.56}},{"doorgr2",{1.56,-1.56}} ,{"doorgr3",{-1.56,1.56}}};
// todo -> find new values -> sampling?
// todo after you found the solution, plan every state separately and the cube movement at the end

void reset_joints(std::shared_ptr<darts::World> &world){
    auto groups = world->getRobot("maze")->getGroups();

    for(auto &a : groups){
        // std::cout << "resetting joint group : " << a.first;
        for(auto &i : a.second)
            world->getRobot("maze")->setJoint(i,0.0);
    }
}

void print_assigned(std::map<std::string,double> assigned){
    for(auto &a : assigned){
        std::cout << "joint name : " << a.first;
        std::cout << " - joint value : " << a.second << std::endl;
    }
}

bool plan_one(std::shared_ptr<darts::World> world, std::string gr_name, std::vector<double> goal_joint_value,
              darts::Window &window){
    darts::PlanBuilder builder(world);

    builder.addGroup("maze", gr_name);

    std::vector<double> start_config(goal_joint_value.size(), 0.0);
    builder.setStartConfiguration(start_config);
    auto idk = builder.getStartConfiguration();
    builder.initialize();

    auto goal = builder.getGoalConfiguration(goal_joint_value);
    builder.setGoal(goal);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto rrt = std::make_shared<ompl::geometric::RRTConnect>(builder.info,true);
    builder.ss->setPlanner(rrt);
    builder.setup();

    ompl::base::PlannerStatus solved = builder.ss->solve(10.0);

    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {

        RBX_INFO("Found solution!");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        window.animatePath(builder,builder.getSolutionPath());
        return true;

    }
    else{
        RBX_WARN("No solution found");

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        reset_joints(world);
        return false;
    }


}



bool move_cube (std::map<std::string,double> assignment, std::shared_ptr<darts::World> world,
                darts::Window &window){
    std::vector<double> goal_state_vector;

    RBX_INFO("Trying following assignment: ");
    print_assigned(assignment);

    for(auto const &a : assignment){
        std::vector<double> goal_vec;
        goal_vec.push_back(a.second);
        if(!plan_one(world,a.first,goal_vec, window))
            return false;
    }
    std::vector<double> goal_vec;
    goal_vec.push_back(0.0);
    goal_vec.push_back(0.55);
    if(!plan_one(world,"cube_gr",goal_vec,window))
        return false;

    reset_joints(world);

    return true;

}

bool find_in_vector(std::vector<std::string> vec, std::string str){
    BOOST_FOREACH(std::string s, vec){
                    if(s == str)
                        return true;
                }
    return false;
}

bool consistent (std::map<std::string,double> assignment,std::shared_ptr<darts::World> world, darts::Window &window){
    std::vector<double> goal_state_vector;

    RBX_INFO("Trying following assignment: ");
    print_assigned(assignment);

    for(auto const &a : assignment){
        std::vector<double> goal_vec;
        goal_vec.push_back(a.second);
        if(!plan_one(world,a.first,goal_vec, window))
            return false;
    }
    reset_joints(world);

    return true;


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

bool backtracking(std::map<std::string,double> &assigned,std::shared_ptr<darts::World> world, darts::Window &window ){
    if(move_cube(assigned,world,window)){
        print_assigned(assigned);
        return true;
    }

    std::map<std::string,std::vector<double>> var = select_unassigned_joint(assigned);
    if(var.size() == 0)
        return false;
    BOOST_FOREACH(double value, var.begin()->second){ // -1.57, 1.57
                    std::map<std::string,double> temp_assigned = assigned;
                    temp_assigned.insert(std::make_pair(var.begin()->first,value));
                    if(consistent(temp_assigned,world,window)){
                        assigned = temp_assigned;
                        auto result = backtracking(assigned,world,window);
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
                                            "/home/serboba/rb_ws/devel/lib/robowflex_dart/maze_sep.srdf");


    auto maze_name = maze_dart->getName();
    auto world = std::make_shared<darts::World>();

    world->addRobot(maze_dart);

    darts::Window window(world);

    window.run([&] {


        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        std::map<std::string,double> assigned;
        std:: cout << backtracking(assigned,world,window) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        move_cube(assigned,world,window);
    });

    return 0;
}