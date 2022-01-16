//
// Created by serboba on 16.01.22.
//
#include <robowflex_dart/planning.h>

#include <robowflex_library/log.h>

using namespace robowflex::darts;
#include <fstream>


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
            for (int j = 0; j < rspace->getDimension(); j++) {
                if (temp_values.at(j) == temp_group(index_it)) {
                    group_index.push_back(j);
                    index_it++;
                }
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

    result.push_back(group_vec);
    result.insert(result.end(),diff_vec.begin(),diff_vec.end());
    return result;

}


bool PlanBuilder::getIntermediateState(const ompl::base::State *from,std::vector<double> to,
                                 ompl::base::State *state, std::vector<int> index_group){
    std::vector<double> s_new_v;
    ompl::base::State *s_new = space->allocState();

    space->copyToReals(s_new_v,from);
    for(int j = 0; j< index_group.size(); j++){
        s_new_v[index_group[j]] = to[index_group[j]];
    }
    space->copyFromReals(s_new,s_new_v);
    space->copyState(state,s_new);

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

        do
        {
            std::vector<ompl::base::State *> temp_states;
            temp_states.push_back(states_new.back());

            for(int j = 0; j< diff_indices.size(); j++)
            {
                ompl::base::State *s_new = info->allocState();
                if(!getIntermediateState(temp_states.back(),s_to,s_new,diff_indices.at(j)))
                    RBX_WARN("ERR: couldn't create new state");

                if(info->checkMotion(temp_states.back(),s_new) && info->isValid(s_new))
                {
                    temp_states.push_back(s_new);
                    if(j+1 == diff_indices.size())
                    {
                        isolated_sub_path=temp_states;
                        iso_success = true;
                    }
                }
                else
                {
                    //std::cout << "ERR : FOUND NEW STATE IS NOT VALID OR NO MOTION POSSIBLE,TRYING NEXT PERMUTATION" << std::endl;
                    temp_states.clear();
                    break;
                }
            }
        }while(std::next_permutation(diff_indices.begin(),diff_indices.end()),iso_success!=true);

        states_new.insert(states_new.end(),isolated_sub_path.begin(),isolated_sub_path.end());  // add the founded sub-isolated path

    }

    std::ofstream fs("maze3doors_pathsep.txt");
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