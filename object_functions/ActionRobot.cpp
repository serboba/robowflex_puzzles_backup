//
// Created by serboba on 11.03.22.
//

#include <robowflex_dart/ActionRobot.h>
#include <iostream>
#include <string>
#include <stdio.h>


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

}

void translateActions(std::vector<std::pair<int,std::vector<double>>> actions_)
{

}
void getActionsFromPath(std::string filename,std::vector<std::vector<int>> group_indices)
{

    std::string str = "path_result/" + filename + ".txt";
    std::ifstream inFile(str); // CHANGE W FILENAME
    std::string gr_name, num;
    std::vector<std::vector<double>> path;

    if (inFile.is_open())
    {
        std::string line;
        while (std::getline(inFile, line))
        {
            std::vector<double> state;
            std::stringstream ss(line);
            if(line.size() < 2)
                break;

            for (int i = 0; i < 6; i++)
            {
                std::getline(ss, num, ' ');
                state.push_back(stod(num));
            }
            path.push_back(state);
        }
    }
    std::vector<std::pair<int,std::vector<double>>> actions_;

    for(int i = 1 ; i < path.size(); i++)
    {
        int actionGroup = indexToGroup(path.at(i-1),path.at(i),group_indices);

        std::vector<double> action_vals;

        for(auto &gr_index : group_indices.at(actionGroup))
        {
            action_vals.push_back(path.at(i).at(gr_index));
        }
        actions_.push_back(std::make_pair(actionGroup,action_vals));

    }

    std::cout << "test"<<std::endl;
    translateActions(actions_);
}


