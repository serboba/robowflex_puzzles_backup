//
// Created by serboba on 16.01.22.
//
#include <robowflex_dart/planning.h>

#include <robowflex_library/log.h>

using namespace robowflex::darts;
#include <fstream>


#include <boost/math/special_functions/factorials.hpp>





bool PlanBuilder::getIntermediateState(const ompl::base::State *from,std::vector<double> to,
                                       ompl::base::State *state, std::vector<int> index_group){
    std::vector<double> s_new_v;
    space->copyToReals(s_new_v,from);

    for(int l = 0; l< index_group.size(); l++){
        s_new_v[index_group[l]] = to[index_group[l]];
    }

    auto *as = state->as<StateSpace::StateType>();
    for(size_t i = 0; i <to.size(); i++)
        as->values[i] = s_new_v.at(i);

    return true;

}


bool PlanBuilder::getIntermediateState(const ompl::base::State *from,const ompl::base::State *to,
                                       ompl::base::State *state, std::vector<int> index_group){
    std::vector<double> to_;
    space->copyToReals(to_,to);
    return getIntermediateState(from,to_,state,index_group);
}

/*
void PlanBuilder::getIntermediateState(std::vector<double> from,std::vector<double> to,
                                       ompl::base::State *state, std::vector<int> index_group){
    std::vector<double> s_new_v;
    ompl::base::State *s_new = space->allocState();

    s_new_v = from;
    for(int l = 0; l< index_group.size(); l++){
        s_new_v[index_group[l]] = to[index_group[l]];
    }
    space->copyFromReals(state,s_new_v);

}
*/
std::vector<int> PlanBuilder::getChangedIndices(std::vector<double> s_from, std::vector<double> s_to){

    std::vector<int> indices;
    for(int i = 0; i < grouped_indices.size() ; i++){
        for(auto const &index_in_gr : grouped_indices.at(i)){
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10){
                indices.push_back(i);
                break;
            }
        }
    }
    return indices;
}


std::vector<int> PlanBuilder::getChangedIndices(const ompl::base::State *from,const ompl::base::State *to) {
    std::vector<double>s1,s2;
    space->copyToReals(s1,from);
    space->copyToReals(s2,to);
    return getChangedIndices(s1,s2);

}


void PlanBuilder::pathIsolateStates(ompl::geometric::PathGeometric *path)
{

    std::vector<ompl::base::State *> states = path->getStates();
    ompl::geometric::PathGeometric path_new(info);
    std::vector<ompl::base::State *> &states_new = path_new.getStates();

    states_new.push_back(states[0]);

    for(int i = 0; i < path->getStateCount()-1; i++)
    {
        std::vector<double> s_from,s_to;
        space->copyToReals(s_from,states[i]);
        space->copyToReals(s_to,states[i+1]);

        std::vector<ompl::base::State *> isolated_sub_path;
        std::vector<int> diff_indices = getChangedIndices(s_from, s_to);
        bool iso_success = false;

        int valid_path_check = 1;
        for(int j = 1; j <= diff_indices.size(); j++)
            valid_path_check *= j;

        int counter = 0;
        do
        {
            std::vector<ompl::base::State *> temp_states;
            ompl::base::State * start_state = info->allocState();
            space->copyState(start_state, states_new.back());
            for(size_t j = 0; j< diff_indices.size(); j++)
            {
                ompl::base::State *s_new = info->allocState();
                if(!getIntermediateState(start_state, s_to, s_new, grouped_indices.at(diff_indices.at(j)))){
                    std::cout << "ERR: no state could be created" << std::endl;
                    break;
                }

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
                // TODO OKAY, WE GO ONE STEP BACK AND INTERPOLATE TO THE POINT AGAIN, DONT ADD SUBPATH
                RBX_WARN("PATH HAS INVALID STATES OR IS DAMAGED. TRIED ALL COMBINATIONS BUT NO ISOLATION SEQUENCE WAS FOUND.");
                if(info->checkMotion(states[i],states[i+1]))
                    std::cout << "motion WAS VALID" << std::endl;
                std::cout << "ABORT" << std::endl;
                return;
            }
            info->freeState(start_state);
        }while(std::next_permutation(diff_indices.begin(),diff_indices.end()),iso_success!=true);

        states_new.insert(states_new.end(),isolated_sub_path.begin(),isolated_sub_path.end());  // add the founded sub-isolated path

    }

    std::ofstream fs("maze_sol_pathiso.txt");
    path_new.printAsMatrix(fs);
    std::cout << "isolated path" << std::endl;

    //path_new.clear();
    //return *path_new;
    repairIsoPath(path_new);
    //  repairIsoPath(path_new);
    //  repairIsoPath(path_new);

}




std::vector<ompl::base::State *> PlanBuilder::buildIntermediateStates(ompl::base::State *start,
                                                                      std::vector<std::pair<int,ompl::base::State * >> stack_) {

    std::vector<ompl::base::State *> merge_;

    for(size_t i = 0 ; i < stack_.size(); i++)
    {
        ompl::base::State * sub_state = info->allocState();
        getIntermediateState(start,stack_.at(i).second,sub_state,grouped_indices.at(stack_.at(i).first));
        space->printState(start);
        space->printState(stack_.at(i).second);
        space->printState(sub_state);
        std::cout << "---BUILD" << std::endl;
        //todo valid motion check
        if(!info->checkMotion(start,sub_state)){
            OMPL_WARN("----------ERRRR NOT VALID MOTION");
            break;
        }
        merge_.push_back(sub_state);
        start = sub_state;
    }

    return merge_;
}

std::vector<ompl::base::State *> PlanBuilder::buildIntermediateStates(ompl::base::State *from,
                                                                      std::vector<std::pair<int,ompl::base::State * >> prio_,
                                                                      std::vector<std::pair<int,ompl::base::State * >> stack_){
    int start_index;

    std::vector<ompl::base::State *> merge_1;
    std::vector<ompl::base::State *> merge_2;
    std::vector<ompl::base::State *> empty_;


    if(prio_.size()!=0){
        merge_1 = buildIntermediateStates(from,prio_); // todo stack states mit changedindex maybe?
        std::cout << "merge 1 " << std::endl;
        //
        if(!stack_.empty() && !merge_1.empty())
        {

            merge_2 =buildIntermediateStates(merge_1.back(),stack_);
            std::cout << "merge 2 " << std::endl;
        }

    }else if(stack_.size()!= 0){
        merge_1 = buildIntermediateStates(from,stack_);

    }else{
        OMPL_WARN("-------------ERRRRRRR BOTH VECTORS SIZE ZERO");
    }

    if(merge_1.size()>0)
    {
        if(merge_2.size()>0){
            merge_1.insert(merge_1.end(),merge_2.begin(),merge_2.end());
            return merge_1;
        }
        else{
            return merge_1;
        }

    }else{
        return merge_2; // even if nothings inside jus return
    }

    //

}

int PlanBuilder::getChangedIndex(const ompl::base::State *from,const ompl::base::State *to){
    return getChangedIndices(from,to).at(0);
}

bool PlanBuilder::repairIsoPath(ompl::geometric::PathGeometric &mainPath){
    OMPL_INFORM("Repairing path (with states cost 1) ----------------------------------");

    // Error check
    if (mainPath.getStateCount() < 2)
    {
        OMPL_ERROR("Cannot repair a path with less than 2 states");
        return false;
    }

    // Loop through every pair of states and make sure path is valid.
    // If not, replan between those states
    std::cout << "mainpath count" << mainPath.getStateCount() <<std::endl;

    int prev_changed_index = getChangedIndices(mainPath.getState(0),mainPath.getState(1)).at(0);
    space->printState(mainPath.getState(0));
    space->printState(mainPath.getState(1));

    for (std::size_t toID = 1; toID < mainPath.getStateCount()-1; ++toID)
    {
        std::size_t fromID = toID - 1;  // this is our last known valid state
        ompl::base::State *fromState = mainPath.getState(fromID);
        ompl::base::State *toState = mainPath.getState(toID);


        std::cout << "new from toid : " << fromID << ", " << toID << std::endl;
        std::cout << "current prev index " << prev_changed_index << std::endl;

        // Check path between states
        if (getChangedIndex(fromState,toState) != prev_changed_index) // wechsel gefunden
        {
            if(getChangedIndices(fromState,toState).size()>1)
                OMPL_DEBUG("ERRR MORE THAN ONE INDEX CHANGED?");
            // Search until next valid STATE with the same changed index end, VON door1, cube, cube, door1, door1, BIS HIER door2
            // oder zb door1, cube, door1, cube dann VON door1, cube, BIS HIER door1, versuche doors zu mergen und stack cube
            // bis also der veränderte index nicht mehr derselbe ist
            std::size_t subsearch_id = toID;


            int index_id = getChangedIndex(fromState,toState);
            std::cout << "not matching index : " << index_id << std::endl;

            std::vector<std::pair<int,ompl::base::State * >> prio_states;
            std::vector<std::pair<int,ompl::base::State * >> stack_states;

            ompl::base::State *new_to;
            ompl::base::State *new_from;
            OMPL_DEBUG("Searching for next same index till it ends, %d to %d had to much cost out  %d total "
                       "states",
                       fromID, toID, mainPath.getStateCount());

            new_to = mainPath.getState(subsearch_id);
            new_from = mainPath.getState(subsearch_id-1);
            stack_states.push_back(std::make_pair(index_id,new_to));
            space->printState(new_to);
            space->printState(new_from);
            std::cout << "---------------------" << std::endl;
            while(getChangedIndex(new_from,new_to) != prev_changed_index && subsearch_id < mainPath.getStateCount()-1){
                subsearch_id++;
                new_to = mainPath.getState(subsearch_id);
                new_from = mainPath.getState(subsearch_id-1);

                space->printState(new_to);
                space->printState(new_from);
                std::cout << "---------------------" << std::endl;
                if(getChangedIndex(new_from,new_to)!=prev_changed_index) {
                    stack_states.push_back(std::make_pair(getChangedIndex(new_from,new_to),new_to));
                }
                else {

                    prio_states.push_back(std::make_pair(getChangedIndex(new_from,new_to),new_to));
                    break;
                }
            }
            //todo lösch last elem vom stack
            while(getChangedIndex(new_from,new_to) == prev_changed_index && subsearch_id < mainPath.getStateCount()-1){
                new_from = mainPath.getState(subsearch_id-1);
                new_to = mainPath.getState(subsearch_id);

                space->printState(new_to);
                space->printState(new_from);
                std::cout << "---------------------" << std::endl;
                if(getChangedIndex(new_from,new_to)==prev_changed_index) {

                    prio_states.push_back(std::make_pair(getChangedIndex(new_from,new_to),new_to));
                }
                else {
                    break;
                }
                subsearch_id++;

            }

            // Check if we ever found a next state that is valid
            if (subsearch_id >= mainPath.getStateCount())
            {
                // We never found a valid state to plan to, instead we reached the goal state and it too wasn't
                // valid. This is bad.
                // I think this is a bug.
                OMPL_ERROR("No state was found valid in the remainder of the path. Invalid goal state. This "
                           "should not happen.");
                return false;
            }
            toID = subsearch_id-1;

            // Plan between our two valid states
            ompl::geometric::PathGeometric newPathSegment(info);

            // Not valid motion, replan
            OMPL_DEBUG("Planning from %d to %d", fromID, toID);
            std::vector<ompl::base::State * > newPathStates;
            if (stack_states.size() != 0 || prio_states.size() != 0)
            {
                newPathStates = buildIntermediateStates(fromState,prio_states,stack_states);
            }

            std::cout << "-----------NEW SUB PATH" << std::endl;
            for(auto st : newPathStates){
                space->printState(st);
                newPathSegment.append(st);
            }
            // TODO make sure THAT WE CUT IF NO VALID MOTION, MEANING -> TO ID TO THE LAST VALID MOTION PLACE

            // Reference to the path
            if(newPathStates.empty()){
                std::cout<< " old prev index : " <<prev_changed_index << std::endl;
                auto  prev_changed = getChangedIndices(mainPath.getState(fromID),mainPath.getState(fromID+1));
                prev_changed_index = getChangedIndices(mainPath.getState(fromID),mainPath.getState(fromID+1)).at(0);
                // toID++;
                // toID = fromID+1;
                std::cout<< " new prev index : " <<prev_changed_index << ", toid :"  << toID<< std::endl;
                std::cout<< "NO VALID MOTION FOUND, GOING TO NEXT INDEX" << std::endl;

                continue;
            }
            std::vector<ompl::base::State *> &primaryPathStates = mainPath.getStates();

            // Remove all invalid states between (fromID, toID) - not including those states themselves
            while (fromID != toID - 1)
            {
                OMPL_INFORM("Deleting state %d", fromID + 1);
                primaryPathStates.erase(primaryPathStates.begin() + fromID + 1);
                --toID;  // because vector has shrunk
            }

            // Insert new path segment into current path
            OMPL_DEBUG("Inserting new %d states into old path. Previous length: %d",
                       newPathSegment.getStateCount() - 2, primaryPathStates.size());

            // Note: skip first and last states because they should be same as our start and goal state, same as
            // `fromID` and `toID`
            std::cout << newPathSegment.getStateCount() << std::endl;
            for (std::size_t i = 1; i < newPathSegment.getStateCount() - 1; i++)
            {
                std::size_t insertLocation = toID + i-1;
                OMPL_DEBUG("Inserting newPathSegment state %d into old path at position %d", i, insertLocation);
                primaryPathStates.insert(primaryPathStates.begin() + insertLocation,
                                         info->cloneState(newPathSegment.getStates()[i]));
            }
            // primaryPathStates.insert( primaryPathStates.begin() + toID, newPathSegment.getStates().begin(),
            // newPathSegment.getStates().end() );
            OMPL_DEBUG("Inserted new states into old path. New length: %d", primaryPathStates.size());

            // Set the toID to jump over the newly inserted states to the next unchecked state. Subtract 2
            // because we ignore start and goal
            toID = toID + newPathSegment.getStateCount() -2 ;
            std::cout << toID << ", " << newPathSegment.getStateCount()<< std::endl;

            std::cout << "last from toid : " << toID-1 << ", " << toID << std::endl;
            OMPL_DEBUG("Continuing searching at state %d", toID);
            if(getChangedIndices(mainPath.getState(toID-1),mainPath.getState(toID)).size()>0){
                std::cout<< " old prev index : " <<prev_changed_index << std::endl;
                prev_changed_index = getChangedIndices(mainPath.getState(toID),mainPath.getState(toID+1)).at(0);

                std::cout<< " new prev index : " <<prev_changed_index << ", toid :"  << toID<< std::endl;
            }
            else{
                break;
            }

        }
    }


    std::ofstream fs("REPAIR.txt");
    mainPath.printAsMatrix(fs);
    std::cout << "REPAIREDpath" << std::endl;

    OMPL_INFORM("Done repairing ---------------------------------");

    return true;

}



std::vector<std::vector<double>> PlanBuilder::getPathReals(std::vector<ompl::base::State *> states)
{
    std::vector<std::vector<double>> state_reals;

    for(int i = 0; i < states.size(); i++)
    {
        std::vector<double> s_;
        space->copyToReals(s_,states[i]);
        state_reals.push_back(s_);
    }
    return state_reals;

}

bool goalStateReached(std::vector<double> goal_, std::vector<double> curr_, std::vector<int> group_index)
{
    bool reach = true;
    for(int i = 0; i < group_index.size(); i++)
    {
        if(goal_.at(i) != curr_.at(group_index.at(i)))
            return false;
    }
    return reach;
}

std::vector<double> extractGoal(std::vector<double> goal_state, std::vector<int> group_index){
    std::vector<double> goal_values;
    for(int i = 0; i < group_index.size(); i++)
    {
        goal_values.push_back(goal_state.at(group_index.at(i)));
    }
    return goal_values;
}



