//
// Created by serboba on 16.01.22.
//
#include <robowflex_dart/planning.h>

#include <robowflex_library/log.h>

using namespace robowflex::darts;
#include <fstream>

#include <boost/math/special_functions/factorials.hpp>

std::vector<std::vector<int>> PlanBuilder::findGroupedIndices(const ompl::base::State *const rfrom) {

    std::vector<std::string> group_names = rspace->getGroups();
    std::vector<std::vector<int>> group_indices;
    ompl::base::State *temp_state = info->allocState();
    space->copyState(temp_state,rfrom);
    std::vector<double> temp_values;

    for(int i = 0; i < space->getDimension(); i++)
        temp_values.push_back(0.0);

    space->copyFromReals(temp_state,temp_values);

    for (int i = 0; i < group_names.size(); i++) {
        int groupDim = rspace->getGroupDimension(group_names[i]);

        if (groupDim > 1) {

            Eigen::VectorXd temp_group(groupDim);
            temp_group << -100.0,-100.0;  // random ???
            rspace->setGroupState(group_names[i],temp_state,temp_group);
            rspace->copyToReals(temp_values, temp_state);
            //FOUND GROUP
            std::vector<int> group_index;
            int index_it = 0;
            for (int j = 0; j < temp_values.size(); j++) {
                if (temp_values.at(j) == temp_group(index_it)) {
                    group_index.push_back(j);
                    index_it++;
                }
                if(index_it ==groupDim)
                    break;
            }
            group_indices.push_back(group_index);
        }
    }
    info->freeState(temp_state);
    return group_indices;
}


std::vector<std::vector<int>> getDiffIndices(std::vector<double> s1,std::vector<double> s2, std::vector<int> group_index){

    std::vector<int> diff_index, group_vec;
    std::vector<std::vector<int>> result, diff_vec;

    for(int j =0; j< s1.size(); j++){
        if(s1.at(j) != s2.at(j))
            diff_index.push_back(j);
    }
    for(int j = 0; j < diff_index.size(); j++){
        if(std::find(group_index.begin(),group_index.end(),diff_index[j]) != group_index.end())
            group_vec.push_back(diff_index[j]);
        else{
            std::vector<int> diff_vec2;
            diff_vec2.push_back(diff_index[j]);
            diff_vec.push_back(diff_vec2);
        }

    }
    if (group_vec.size() >0)
        result.push_back(group_vec);
    result.insert(result.end(),diff_vec.begin(),diff_vec.end());
    return result;

}


bool PlanBuilder::getIntermediateState(const ompl::base::State *from,std::vector<double> to,
                                       ompl::base::State *state, std::vector<int> index_group){
    std::vector<double> s_new_v;
    ompl::base::State *s_new = space->allocState();

    space->copyToReals(s_new_v,from);
    for(int l = 0; l< index_group.size(); l++){
        s_new_v[index_group[l]] = to[index_group[l]];
    }
    space->copyFromReals(s_new,s_new_v);
    //space->copyState(state,s_new);
    space->copyFromReals(state,s_new_v);
    if(space->equalStates(state,from))
        return false;

    return true;

}



ompl::geometric::PathGeometric PlanBuilder::pathIsolateStates(ompl::geometric::PathGeometric path){

    std::vector<ompl::base::State *> &states = path.getStates();
    std::vector<std::vector<int>> groupIndices = findGroupedIndices(states[0]);
    std::vector<int> group_index = groupIndices.at(0); // TODO for more than one group (groupIndices alrdy. correct -> todo in getDiffIndices()
    //std::vector<int> group_index = {0,1};
    ompl::geometric::PathGeometric path_new = ompl::geometric::PathGeometric(info);
    std::vector<ompl::base::State *> &states_new = path_new.getStates();

    states_new.push_back(states[0]);
    
    for(int i = 0; i < path.getStateCount()-1; i++)
    {
        std::vector<double> s_from,s_to;
        space->copyToReals(s_from,states[i]);
        space->copyToReals(s_to,states[i+1]);

        std::vector<ompl::base::State *> isolated_sub_path;
        std::vector<std::vector<int>> diff_indices = getDiffIndices(s_from,s_to, group_index);
        bool iso_success = false;

        int valid_path_check = 1;
        for(int j = 1; j <= diff_indices.size(); j++)
            valid_path_check *= j;

        int counter = 0;
        do
        {
            std::vector<ompl::base::State *> temp_states;
            ompl::base::State * start_state = space->allocState();
            space->copyState(start_state, states_new.back());
            for(int j = 0; j< diff_indices.size(); j++)
            {
                ompl::base::State *s_new = info->allocState();
                if(!getIntermediateState(start_state, s_to, s_new, diff_indices.at(j))){
                    std::cout << "ERR: no state could be created" << std::endl;
                    break;
                }
                /*
                std::cout << "start" << std::endl;
                space->printState(states[i]);
                space->printState(states[i+1]);
                space->printState(start_state);
                space->printState(s_new);
                std::cout <<"check motion " << info->checkMotion(start_state,s_new) << std::endl;
                std::cout <<"diff size " << diff_indices.size() <<" , j : " << j <<  std::endl;
                std::cout <<"isvalid " << info->isValid(s_new) << std::endl;
                std::cout << "start equal : " << space->equalStates(s_new,states[i+1]) <<  std::endl;
                */
                 if(info->checkMotion(start_state,s_new))
                {
                    temp_states.push_back(s_new);
                    space->copyState(start_state,s_new);
                    if(j+1 == diff_indices.size()){
                        iso_success = true;
                    }
                }
                else
                {
                    temp_states.clear();
                    break;
                }

            }
            counter++;
            if(iso_success){
                isolated_sub_path=temp_states;
            }else if(valid_path_check < counter){
                RBX_WARN("PATH HAS INVALID STATES OR IS DAMAGED. TRIED ALL COMBINATIONS BUT NO ISOLATION SEQUENCE WAS FOUND.");
                std::cout << "ABORT" << std::endl;
                return path_new;
            }
            space->freeState(start_state);
        }while(std::next_permutation(diff_indices.begin(),diff_indices.end()),iso_success!=true);

        states_new.insert(states_new.end(),isolated_sub_path.begin(),isolated_sub_path.end());  // add the founded sub-isolated path

    }

    std::ofstream fs("maze_sol_pathiso.txt");
    path_new.printAsMatrix(fs);
    std::cout << "isolated path" << std::endl;
    return path_new;

}


/*
            for(int j = 0; j< diff_indices.size(); j++)
            {
                ompl::base::State *s_new = info->allocState();
                if(!getIntermediateState(tempStates.back(),s_to,s_new,diff_indices.at(j)))
                    RBX_WARN("ERR: couldn't create new state");
                if(info->checkMotion(tempStates.back(),s_new) && info->isValid(s_new))
                {
                    tempStates.push_back(s_new);
                    if(j+1 == diff_indices.size())
                    {
                        isolatedStates=tempStates;
                        iso_success = true;
                    }
                }
                else
                {
                    //std::cout << "ERR : FOUND NEW STATE IS NOT VALID OR NO MOTION POSSIBLE,TRYING NEXT PERMUTATION" << std::endl;
                    tempStates.clear();
                    break;
                }
            }
 */

/*
bool PlanBuilder::getIsolatedSubpath(std::vector<std::vector<int>> diff_indices, std::vector<ompl::base::State *> &tempStates,std::vector<double> s_to) {
    for (int j = 0; j < diff_indices.size(); j++) {
        ompl::base::State *s_new = info->allocState();
        if (!getIntermediateState(tempStates.back(), s_to, s_new, diff_indices.at(j)))
            RBX_WARN("ERR: couldn't create new state");
        if (info->checkMotion(tempStates.back(), s_new) && info->isValid(s_new)) {
            tempStates.push_back(s_new);
            if (j + 1 == diff_indices.size()) {
                return true; // set iso = temp in other
            }
        } else {
            //std::cout << "ERR : FOUND NEW STATE IS NOT VALID OR NO MOTION POSSIBLE,TRYING NEXT PERMUTATION" << std::endl;
            tempStates.clear();
            return false;
        }
    }
}
*/