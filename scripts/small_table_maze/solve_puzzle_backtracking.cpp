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
#include <robowflex_dart/solution_parser.h>

using namespace robowflex;

static const std::string GROUP = "arm_with_torso";

std::vector<std::pair<std::string,std::vector<double>>> d_values;
std::vector<std::vector<std::pair<std::string,std::vector<double>>>> d_maps;

int d_count = 0;
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

std::vector<std::string> get_joint_names(std::shared_ptr<darts::World> &world){
    std::vector<std::string> names;
    auto groups = world->getRobot("maze")->getGroups();
    for(auto &a : groups){
        if(a.first == "cube_gr1" || a.first == "cube_gr2")
            continue;
        names.push_back(a.first);
    }
    return names;

}


void print_assigned(std::vector<std::pair<std::string,double>> assigned){
    for(auto &a : assigned){
        std::cout << "joint name : " << a.first;
        std::cout << " - joint value : " << a.second << std::endl;
    }
}

bool plan_one(std::shared_ptr<darts::World> world, std::string gr_name, std::vector<double> goal_joint_value,
              darts::Window &window){
    darts::PlanBuilder builder(world);
    if(gr_name == "cube_gr"){
        builder.addGroup("maze", "cube_gr1");
        builder.addGroup("maze", "cube_gr2");
    }
    else
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


bool move_cube (std::vector<std::pair<std::string,double>> assignment, std::shared_ptr<darts::World> world,
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

int find_index_in_vector(std::string str){
    for(int i =0; i< d_values.size(); i++){
        if(str == d_values[i].first)
            return i;
    }
    return -1;
}

bool consistent (std::vector<std::pair<std::string,double>> assignment,std::shared_ptr<darts::World> world, darts::Window &window){
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


std::pair<std::string,std::vector<double>> select_unassigned_joint(std::vector<std::pair<std::string,double>> assigned_){
    std::vector<std::string> assigned_joints;
    for(auto& joints : assigned_){
        assigned_joints.push_back(joints.first);
        std::cout << "Pushing : " << joints.first << "-";
    }
    std::cout <<"\n" << std::endl;

    std::pair<std::string,std::vector<double>> var;
    for(auto & d_var : d_values){
        if(!find_in_vector(assigned_joints,d_var.first)){
            var = std::make_pair(d_var.first,d_var.second);
            return var;
        }
    }
}

void pop_element(std::vector<std::pair<std::string,double>> &assigned_){
    /*
     for(auto it = assigned_.begin(); it != assigned_.end(); it++){
         std::cout <<"pop: " <<  it->first << std::endl;
     }
     auto it = assigned_.end();
     it--;
 */
    assigned_.erase(std::prev(assigned_.end()));
}


bool backtracking(std::vector<std::pair<std::string,double>> &assigned,std::shared_ptr<darts::World> world, darts::Window &window ){
    if(move_cube(assigned,world,window)){
        print_assigned(assigned);
        return true;
    }
    print_assigned(assigned);
    std::pair<std::string,std::vector<double>> var = select_unassigned_joint(assigned);
    if(var.first != ""){
        std::cout <<  "ALL CHILD NODES OF LAST JOINT ASSINGED" << std::endl;
        print_assigned(assigned);
    }
    if(var.first != "" ){
        for(double value : var.second){ // -1.57, 1.57
            //temp_assigned.insert(std::make_pair(var.begin()->first,value));
            std::vector<std::pair<std::string,double>> temp_assigned = assigned;
            temp_assigned.push_back(std::make_pair(var.first,value));

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
    }

    return false;

}

std::vector<std::vector<std::pair<std::string,std::vector<double>>>> get_permutations(){

    std::vector<std::vector<std::pair<std::string,std::vector<double>>>> d_vals; //domain permutations
    std::vector<int> perm_num;
    for (int i = 0; i < d_values.size() ; i++) {
        perm_num.push_back(i);
    }

    std::vector<std::string> joint_names;
    for(auto domain: d_values){
        joint_names.push_back(domain.first);
    }

    do {
        std::vector<std::pair<std::string,std::vector<double>>> temp_map;
        for(int i = 0; i < joint_names.size() ; i++ ){

            int index = find_index_in_vector(joint_names[i]);
            temp_map.push_back(std::make_pair(joint_names[i],d_values[index].second));

            std::cout << "ADDING JOINT : " << joint_names[i] << " - " << d_values[index].second[0] << "-"<< d_values[index].second[1] << "-" << d_values[index].second[2] <<std::endl;
        }
        std::cout << "CURRENT JOINT NAME COMBINATION" << joint_names[0] << "-" << joint_names[1] << "-" << joint_names[2] << std::endl;
        d_vals.push_back(temp_map);
    }while(std::next_permutation(joint_names.begin(),joint_names.end()));

    return d_vals;
}


bool solve_backtracking(std::shared_ptr<darts::World> world, std::vector<std::pair<std::string,double>> &assigned, darts::Window &window){

    std::vector<std::string> joint_names = get_joint_names(world);
    std::cout << joint_names[0] << "-" <<joint_names[1] << "- " << joint_names[2] << std::endl;
    // auto sasfjsd = d_maps;
    //  if(backtracking(assigned,world,window)){
    //      std::cout << "FOUND SOLUTION FIRST TRY" << std::endl;
    //     return true;
    // }else{
    std::cout << "NO SOLUTION FOUND FIRST ATTEMPT, TRYING DIFFERENT PERMUTATIONS" << std::endl;
    for(int i = 0 ; i <d_maps.size() ; i++){ // i = SET PERMUTATION YOU WANT TO BEGIN WITH
        d_values = d_maps[i];
        std::cout << d_values.begin()->first << "-" << std::endl;

        //d_values = get_domain_values(joint_names);
        if (backtracking(assigned,world,window)){
            std::cout << "FOUND SOLUTION ! " << std::endl;
            return true;
        }

    }
    // }
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


    window.run([&] {
        std::vector<std::string> joint_names =get_joint_names(world);
        d_values = get_domain_values(joint_names);
        d_maps = get_permutations();

        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        std::vector<std::pair<std::string,double>> assigned;
        solve_backtracking(world, assigned,window);
        std::this_thread::sleep_for(std::chrono::milliseconds(200000));
        // move_cube(assigned,world,window);
    });

    return 0;
}
