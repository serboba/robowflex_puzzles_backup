//
// Created by serboba on 11.03.22.
//

#include <robowflex_dart/ActionRobot.h>
#include <iostream>
#include <string>
#include <stdio.h>


boost::filesystem::path pp_(boost::filesystem::current_path().parent_path().parent_path().parent_path());
const std::string abs_path = pp_.string() + "/src/robowflex/robowflex_dart/include/io/";



int indexToGroup(std::vector<double>s_from,std::vector<double>s_to,std::vector<std::vector<int>> group_indices)
{
        for(size_t i = 0; i < group_indices.size() ; i++)
        {
            for(auto const &index_in_gr : group_indices.at(i))
            {
                if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10)
                    return i;
            }
        }
        return -1;
}

void translateActions(std::vector<ActionP> &actions_, std::vector<Object> objects_, std::vector<ActionR> &actions_robot)
{
    for(auto &path_action : actions_)
    {
        auto obj = objects_.at(path_action.group_index);
        Vector3d pos;
        Vector3d rpy;
        pos << 0.0,0.0,0.0;
        rpy << 0.0,0.0,0.0;

        for(size_t i = 0; i < path_action.vals.size(); i++)
        {
            if(obj.joints.joints.at(i).type == prismatic)
                pos(obj.joints.joints.at(i).direction) = path_action.vals[i];
            else
                rpy(obj.joints.joints.at(i).direction) = path_action.vals[i];
        }
        actions_robot.push_back(ActionR(pos,rpy,obj.link.name,path_action.group_index));
    }


    Vector3d rpy_it;
    Vector3d pos_it;

    for(size_t i = 0; i < objects_.size() ; i++)
    {
        rpy_it.setZero();
        pos_it.setZero();
        for(auto &action_ : actions_robot)
        {
            if(action_.obj_index == i)
            {
                action_.pos = action_.pos - pos_it;
                action_.rpy = action_.rpy - rpy_it;

                pos_it += action_.pos;
                rpy_it += action_.pos;
            }
        }
    }
}

void getActionsFromPath(std::string filename,std::vector<std::vector<int>> group_indices,std::vector<ActionP> &actions_)
{

    std::string str = abs_path+"path_result/" + filename + ".txt";
    std::ifstream inFile(str); // CHANGE W FILENAME
    std::string gr_name, num;
    std::vector<std::vector<double>> path;

    int dim = 0;
    for(auto &g : group_indices)
        for(auto &gg : g)
            dim++;

    if (inFile.is_open())
    {
        std::string line;
        while (std::getline(inFile, line))
        {
            std::vector<double> state;
            std::stringstream ss(line);
            if(line.size() < 2)
                break;

            for (int i = 0; i < dim; i++)
            {
                std::getline(ss, num, ' ');
                state.push_back(stod(num));
            }
            path.push_back(state);
        }
    }

    for(size_t i = 1 ; i < path.size(); i++)
    {

        int actionGroup = indexToGroup(path.at(i-1),path.at(i),group_indices);

        std::vector<double> action_vals;

        for(auto &gr_index : group_indices.at(actionGroup))
        {
            action_vals.push_back(path.at(i).at(gr_index));
        }
        actions_.push_back(ActionP(actionGroup,action_vals));

    }
    std::cout << "test"<<std::endl;
}


