/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
        *  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
        *     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
        *     disclaimer in the documentation and/or other materials provided
        *     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
        *     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
        *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
        *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
        *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
                                                                     *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
        *  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

//
// Created by serboba on 22.01.22.
//

#include <robowflex_dart/RRTnew.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"

ompl::geometric::RRTnew::RRTnew(const base::SpaceInformationPtr &si, std::vector<std::vector<int>> gr_indices, bool addIntermediateStates,
                                bool useIsolation)
        : base::Planner(si, addIntermediateStates ? "RRTnewIntermediate" : "RRTnew")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;
    group_indices = gr_indices;

    Planner::declareParam<double>("range", this, &RRTnew::setRange, &RRTnew::getRange, "0.:1.:10000.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTnew::setIntermediateStates,
                                &RRTnew::getIntermediateStates, "0,1");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;
    useIsolation_ = useIsolation;
    addPlannerProgressProperty("best cost DOUBLE", [this] { return bestCostProperty(); });
    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
}

ompl::geometric::RRTnew::~RRTnew()
{
    freeMemory();
}

void ompl::geometric::RRTnew::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    opt_ = std::make_shared<ompl::base::IsoManipulationOptimization>(si_,group_indices);
    pdef_->setOptimizationObjective(opt_);

    bestCost_ = opt_->infiniteCost();


    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });


}

void ompl::geometric::RRTnew::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RRTnew::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

    if(nn_)
        nn_->clear();
}
void ompl::geometric::RRTnew::getChangedIndices(const base::State* rfrom, const base::State* rto,
                                                std::vector<int> &indices_) const{


    std::vector<double> s1,s2;
    si_->getStateSpace()->copyToReals(s1,rfrom);
    si_->getStateSpace()->copyToReals(s2,rto);

    for (size_t i= 0; i < s1.size() ; i++) {
        if (abs(s1.at(i) - s2.at(i)) > 0.0001) {
            indices_.push_back(i);
        }
    }

}

std::vector<std::vector<int>> ompl::geometric::RRTnew::getChangedGroups(const std::vector<double> &from_,const std::vector<double> &to_){
    std::vector<std::vector<int>> groups;
    for(auto group : group_indices){
        std::vector<int> gr_;
        for(auto index : group){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                gr_.push_back(index);
            }
        }
        if(gr_.size() >0)
            groups.push_back(gr_);
    }
    return groups;
}


std::vector<ompl::base::State *> ompl::geometric::RRTnew::buildIsoStates(const std::vector<double> &from_,const std::vector<double> &to_,
                                                                         std::vector<std::vector<int>> &changed_index_groups ){

    std::vector<ompl::base::State *> isolateResult;
    std::vector<double> intermediate_st(from_.size());
    intermediate_st = from_;

    for(auto v : changed_index_groups){
        for(int i : v){
            intermediate_st[i] = to_[i];
        }
        base::State *temp = si_->allocState();
        si_->getStateSpace()->copyFromReals(temp,intermediate_st);
        isolateResult.push_back(temp);
    }
    return isolateResult;
}

std::vector<std::vector<int>> ompl::geometric::RRTnew::reorderGroup(const std::vector<int> &changed_indices,
                                                                    const std::vector<std::vector<int>> &groups){

    std::vector<std::vector<int>> changed_index_groups;
    for(size_t i = 0; i < changed_indices.size() ; i++){
        std::vector<int> group;
        for(auto const group_ : groups){
            std::vector<int> group_obj;
            if((std::find(group_.begin(),group_.end(),changed_indices.at(i)) != group_.end())){
                for(size_t j = 0; j < group_.size(); j++){
                    group_obj.push_back(group_.at(j));
                }
                i+= group_.size();

                changed_index_groups.push_back(group_obj);
                break;
            }
        }
        if(i < changed_indices.size()){
            group.push_back(changed_indices.at(i));
            changed_index_groups.push_back(group);
        }
    }
    return changed_index_groups;
}

std::vector<ompl::base::State *> ompl::geometric::RRTnew::isolateStates(const base::State* rfrom, const base::State* rto){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);


    //std::vector<int> changed_indices(ompl::base::IsoManipulationOptimization::changedIndex(from_,to_,changed_indices));
    std::vector<int> changed_indices;
    getChangedIndices(rfrom,rto,changed_indices);

    std::vector<std::vector<int>> groups = getChangedGroups(from_,to_);

    std::vector<std::vector<int>> changed_index_groups = reorderGroup(changed_indices,groups);

    return buildIsoStates(from_,to_,changed_index_groups);

}


ompl::geometric::RRTnew::Motion * ompl::geometric::RRTnew::createNewMotion(const base::State *st, ompl::geometric::RRTnew::Motion *premotion){
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, st);

    motion->parent = premotion;
    motion->root = premotion->root;
    motion->cost = opt_->motionCost(premotion->state,st);
   // motion->index_changed = getChangedIndex(premotion->state,st);
    if(!getChangedIndexGroups(premotion->state,st).empty())
        motion->index_changed = getChangedIndexGroups(premotion->state,st).back();
    else
        motion->index_changed = getChangedIndex(premotion->state,st);
    if(motion->index_changed == premotion->index_changed){
        motion->incCost = opt_->combineCosts(premotion->incCost, base::Cost(0.0,motion->cost.distval()));
    }else{
        motion->incCost = opt_->combineCosts(premotion->incCost,motion->cost);
    }
    return motion;
}

void ompl::geometric::RRTnew::printMotion(Motion * motion)
{
    std::cout << "----Printing motion...: " << std::endl;
    if(motion->parent != nullptr && motion ->parent != NULL)
    {
        std::cout << "Parent mot !nullptr : " << std::endl;
        si_->printState(motion->parent->state);
    }
    std::cout<<"Motion state:" << std::endl;
    si_->printState(motion->state);
    std::cout << "Motion cost : " << motion->cost.value() << ", dist cost : " << motion->cost.distval() << ", index : "<< motion->index_changed  <<std::endl;
    std::cout << "Motion incCost : " << motion->incCost.value() << ",d: " << motion->incCost.distval() << std::endl;

    if(motion->parent != nullptr && motion ->parent != NULL)
    {
        std::cout <<"Motions parent motion cost: " << motion->parent->cost.value() << ", dist cost:" << motion->parent->cost.distval() << std::endl;
        std::cout <<"Motions parent incCost: " << motion->parent->incCost.value() << ", dist cost: " << motion->parent->incCost.distval() << std::endl;
    }
    if(motion->parent != nullptr && motion ->parent != NULL){
        std::cout << "Motions parent changed index: " << motion->parent->index_changed << std::endl;
    }
}



bool ompl::geometric::RRTnew::validMotionCheck(const bool start, const base::State *from_,const base::State *to_){
    bool validmo= start ? si_->checkMotion(from_, to_) :
                  si_->isValid(to_) && si_->checkMotion(to_, from_);

    return validmo;

}

std::vector<ompl::geometric::RRTnew::Motion *> ompl::geometric::RRTnew::getMotionVectors(Motion * mot_)
{
    /* construct the motion vec */
    Motion *solution = mot_;
    std::vector<Motion *> vec;
    while (solution != nullptr)
    {
        vec.push_back(solution);
        solution = solution->parent;
    }

    return vec;
}

void ompl::geometric::RRTnew::constructSolutionPath(Motion * startMotion, Motion * goalMotion, bool save,const base::PlannerTerminationCondition &ptc)
{

    if (startMotion->parent != nullptr)
        startMotion = startMotion->parent;
    else
        goalMotion = goalMotion->parent;

    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

    std::vector<Motion *> mpath1 = getMotionVectors(startMotion);
    std::vector<Motion *> mpath2 = getMotionVectors(goalMotion);


    auto path(std::make_shared<PathGeometric>(si_));
    path->getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i){
        path->append(mpath1[i]->state);
    }
    for (auto &i : mpath2){
        path->append(i->state);
    }
    bestCost_ = base::Cost(getCostPath(path->getStates()));
    pdef_->addSolutionPath(path, false, 0.0, getName());

/*
    if(!save){
        base::Cost maxCost(getCostPath(path->getStates()));
        bestCost_ = maxCost;
        OMPL_DEBUG("SOLVED, STARTING REWIRING, COST BEFORE REWIRING %d:", maxCost);


        int counter =0;

        std::vector<ompl::base::State *> path_temp = path->getStates();
        while(!ptc){
            rewire(path_temp);
            base::Cost temp_cost(getCostPath(path_temp));
            //  std::cout << "temp - max : " << temp_cost.value() << "," << maxCost.value()<< std::endl;
            if(temp_cost.value() <= maxCost.value())
            {
                if(temp_cost.value() == maxCost.value())
                    counter++;
                if(counter ==5)
                    break;
                maxCost = temp_cost;
                bestCost_ = maxCost;
                path->getStates() = path_temp;


            }
            else
            {
                break;
            }
        }

        bestCost_ = maxCost;
        OMPL_DEBUG("SOLVED, FURTHER REWIRING. FINAL cost : %d", getCostPath(path->getStates()));
        pdef_->addSolutionPath(path, false, 0.0, getName());
        //simplifyPath(path->getStates());
    }
    else{
        std::ofstream fs1("mazeBEFORE.txt");
        path->printAsMatrix(fs1);
    }
*/

 }


void ompl::geometric::RRTnew::getIntermediateState(const ompl::base::State *from,const ompl::base::State * to, ompl::base::State *state, int index_group)
{
    std::vector<double> s_new_v,to_;
    si_->getStateSpace()->copyToReals(s_new_v,from);
    si_->getStateSpace()->copyToReals(to_,to);

    std::vector<int> group = group_indices.at(index_group);
    for(size_t i = 0; i< group.size(); i++){
        s_new_v[group.at(i)] = to_[group.at(i)];
    }

    si_->getStateSpace()->copyFromReals(state,s_new_v);
}


int ompl::geometric::RRTnew::getChangedIndex(const ompl::base::State *from,const ompl::base::State * to){

    // setze voraus dass cost 1 zw. from und to, keine equal states

    std::vector<double> s_from,s_to;
    si_->getStateSpace()->copyToReals(s_from,from);
    si_->getStateSpace()->copyToReals(s_to,to);

    for(size_t i = 0; i < group_indices.size() ; i++)
    {
        for(auto const &index_in_gr : group_indices.at(i))
        {
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10)
                return i;
        }
    }
}

std::vector<int> ompl::geometric::RRTnew::getChangedIndexGroups(const ompl::base::State *from,const ompl::base::State * to){

    // setze voraus dass cost 1 zw. from und to, keine equal states

    std::vector<double> s_from,s_to;
    si_->getStateSpace()->copyToReals(s_from,from);
    si_->getStateSpace()->copyToReals(s_to,to);


    std::vector<int> changed_groups_;
    for(size_t i = 0; i < group_indices.size() ; i++)
    {
        for(auto const &index_in_gr : group_indices.at(i))
        {
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10){
                changed_groups_.push_back(i);
                break;
            }
        }
    }
    return changed_groups_;
}


std::vector<ompl::base::State * > ompl::geometric::RRTnew::getStates(std::vector<ompl::geometric::RRTnew::Motion *> motions)
{
    std::vector<ompl::base::State * > states;
    states.reserve(motions.size());

    //std::cout << "GETSTATES SIZE BEFORE : " << motions.size() << std::endl;

    for(auto &i: motions){
        if(!states.empty() && si_->equalStates(i->state,states.back())){
            std::cout << "----STATES EQUAL :" << std::endl;
            //    si_->printState(i->state);
            //   si_->printState(states.back());
            continue;
        }
        states.push_back(i->state);
    }


    // std::cout << "GETSTATES SIZE AFTER : " << states.size() << std::endl;
    return states;
}

ompl::geometric::RRTnew::Motion * ompl::geometric::RRTnew::getReConnectedMotions(std::vector<ompl::base::State*> states)
{

    auto *root_motion = new Motion(si_);
    root_motion->root = states.back();
    root_motion->state = states.back();
    root_motion->cost = base::Cost(1.0);


    std::vector<ompl::geometric::RRTnew::Motion *> mots_;
    mots_.push_back(root_motion);
    for(size_t i = states.size()-1; i > 0; i--){
        Motion * m = createNewMotion(states.at(i-1),mots_.front());
        mots_.insert(mots_.begin(),m);
    }


    auto test = getStates(getMotionVectors(mots_.front()));

    return mots_.front();

}


ompl::geometric::RRTnew::Motion * ompl::geometric::RRTnew::getReConnectedMotionsGOAL(std::vector<ompl::base::State*> states)
{

    auto *root_motion = new Motion(si_);
    root_motion->root = states.back();
    root_motion->state = states.back();
    root_motion->cost = base::Cost(1.0);


    std::vector<ompl::geometric::RRTnew::Motion *> mots_;
    mots_.push_back(root_motion);
    for(size_t i = states.size()-1; i > 0; i--){
        Motion * m = createNewMotion(states.at(i-1),mots_.front());
        mots_.insert(mots_.begin(),m);
    }

    return mots_.front();

}


std::vector<ompl::base::State * > ompl::geometric::RRTnew::reConnect(ompl::base::State *from,
                                                                     std::vector<std::pair<ompl::base::State *,int >> queue_)
{
    std::vector<ompl::base::State * > merge_;

    for(size_t i = 0; i < queue_.size(); i++)
    {
        ompl::base::State * newEdge = si_->allocState();
        getIntermediateState(from,queue_.at(i).first,newEdge,queue_.at(i).second);
        if(!si_->checkMotion(from,newEdge))
            return {};
        merge_.push_back(newEdge);
        from = newEdge;
    }
    return merge_;
}


std::vector<ompl::base::State * > ompl::geometric::RRTnew::reConnect(ompl::base::State *from,
                                                                     std::vector<std::pair<ompl::base::State *,int >> prio_,
                                                                     std::vector<std::pair<ompl::base::State *,int >> stack_)
{

    std::vector<ompl::base::State * > mergedStates;
    std::vector<ompl::base::State * > queuedStates;

    mergedStates = reConnect(from,prio_);
    if(!mergedStates.empty())
        queuedStates = reConnect(mergedStates.back(),stack_);
    if(mergedStates.empty() || queuedStates.empty())
        return {};

    mergedStates.insert(mergedStates.end(),queuedStates.begin(),queuedStates.end());
    return mergedStates;

}

int ompl::geometric::RRTnew::rewire(std::vector<ompl::base::State *> &mainPath) {
/*
    std::cout<< "INSIDE REWIRE PRINTING STATES ONLY FIRST ---------" << std::endl;
    for(auto &i : mainPath)
        si_->printState(i);
    std::cout<<"------------------END STATES" << std::endl;
*/
    int rewireCount = 0;
    int prev_index = getChangedIndex(mainPath.at(0),mainPath.at(1));
    for(size_t toID = 1; toID < mainPath.size()-1; ++toID){
        size_t fromID = toID-1;

        ompl::base::State *fromState = mainPath.at(fromID);
        ompl::base::State  *toState = mainPath.at(toID);
/*
       std::cout <<"fromstate, tostate, toID :" << fromID << ", "<< toID << std::endl;
        si_->printState(fromState);
        si_->printState(toState);

        std::cout << "indices different: " << getChangedIndex(fromState,toState) << " ? " << prev_index << std::endl;
  */
        if(getChangedIndex(fromState,toState) != prev_index){
            //prev_index = toID; // TODO ??


            int subsearch_id = toID;
            ompl::base::State *subFrom = mainPath.at(subsearch_id-1);
            ompl::base::State  *subTo = mainPath.at(subsearch_id);
            /*
                    std::cout << "indices different: " << getChangedIndex(fromState,toState) << " != " << prev_index << std::endl;
                    std::cout <<"subfrom , subto :" << std::endl;
                    si_->printState(subFrom);
                    si_->printState(subTo);
        */
            std::vector<std::pair<ompl::base::State *,int>> mergeIndices;
            std::vector<std::pair<ompl::base::State *,int>> queueIndices;
            queueIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));

            while (getChangedIndex(subTo,subFrom) != prev_index && subsearch_id < mainPath.size()-1){
                subsearch_id++;

                subFrom = mainPath.at(subsearch_id-1);
                subTo = mainPath.at(subsearch_id);
/*
                si_->printState(subFrom);
                si_->printState(subTo);
                std::cout<< "subsearchid: " << subsearch_id << ",index changed : " << getChangedIndex(subFrom,subTo)  << " prev : " << prev_index<< std::endl;
  */
                if(getChangedIndex(subFrom,subTo) != prev_index){
                    queueIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));
                }else{
                    /*    //start same index again done
                        std::cout << "------------------------" << std::endl;
                        std::cout << "breaking" << std::endl;
    */
                    break;
                }
            }

//            std::cout << "------------------------  QUEUE START" << std::endl;
            while(getChangedIndex(subTo,subFrom) == prev_index && subsearch_id < mainPath.size()-1)
            {
                subFrom = mainPath.at(subsearch_id-1);
                subTo = mainPath.at(subsearch_id);
/*
                si_->printState(subFrom);
                si_->printState(subTo);
                std::cout <<"subsearchid: " << subsearch_id << ", index changed : " << getChangedIndex(subFrom,subTo)  << " prev : " << prev_index<< std::endl;
*/
                if(getChangedIndex(subFrom,subTo) == prev_index)
                    mergeIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));
                else
                    break;
                subsearch_id++;
            }

            //  toID = subsearch_id-1;
            //     std::cout << "size : "<<mergeIndices.size() << "-" << queueIndices.size() << std::endl;
            if(subsearch_id >= mainPath.size())
                OMPL_ERROR("SUB SEARCH ID OVER SIZE ERR");

            std::vector<ompl::base::State * > rewiredConnection;

            if(!mergeIndices.empty() && !queueIndices.empty())
                rewiredConnection = reConnect(fromState,mergeIndices,queueIndices);
            else{
                toID = toID+1;
                if(toID>= mainPath.size()-1)
                    return rewireCount;
                prev_index = getChangedIndex(mainPath.at(toID),mainPath.at(toID+1));

                /*       std::cout << "old state was : "<< std::endl;
                       si_->printState(mainPath.at(fromID));
                       si_->printState(mainPath.at(fromID+1));
                       std::cout << "new state is : "<< std::endl;
                       si_->printState(mainPath.at(toID));
                       si_->printState(mainPath.at(toID+1));
                       std::cout << "fromID - toID : "<< fromID << ", new: " << toID <<"-" << (toID+1) << std::endl;

                       std::cout << "new prev index "<< prev_index << std::endl;
                   */
            }

            if(rewiredConnection.empty()){
                continue;
            }
            // toID original bis
            int end_interval = subsearch_id-1;
            //  OMPL_DEBUG("REWIRING %d STATES", rewiredConnection.size());
            rewireCount+=rewiredConnection.size();
            int l = 0;
            for(size_t j = toID; j <toID+rewiredConnection.size(); j++)
            {
                mainPath[j] = rewiredConnection.at(l);
                //     std::cout<< "MAIN PATH AFTER REWIRE AT " << j << std::endl;
                //     si_->printState(mainPath[j]);
                l++;
            }

            //todo set prev index
            // todo set toid to end of merged interval in this case
            //  std::cout << "to id after rewire before :" << toID;
            toID = toID+mergeIndices.size()+1; // check??
            // std::cout << "-" << toID << std::endl;

            if(toID>= mainPath.size()-1)
                return rewireCount;
            //  std::cout << "previndex rewire before : " << prev_index;
            prev_index = getChangedIndex(mainPath.at(toID),mainPath.at(toID+1));
            //  std::cout << "-" << prev_index << std::endl;
        }

    }

    return rewireCount;

}

int ompl::geometric::RRTnew::getCostPath(std::vector<ompl::base::State * > states_)
{
    int prev_index = getChangedIndex(states_.at(0),states_.at(1));
    int cost = 1;
    for (int i = 1; i < states_.size()-1 ; ++i) {
        //  si_->printState(states_.at(i));
        //  si_->printState(states_.at(i+1));
        if(prev_index != getChangedIndex(states_.at(i),states_.at(i+1)))
        {
            cost++;
            prev_index = getChangedIndex(states_.at(i),states_.at(i+1));
        }

    }

    return cost;
}



int ompl::geometric::RRTnew::getCostPath(ompl::geometric::RRTnew::Motion * mot_)
{
    return getCostPath(getStates(getMotionVectors(mot_)));
}

int ompl::geometric::RRTnew::rewireTree(Motion *startMotion, Motion *goalMotion)
{
    //do start motion and goalmotion separated,
/*
    if (startMotion->parent != nullptr)   //go one step back??
        startMotion = startMotion->parent;
    else
        goalMotion = goalMotion->parent;
*/

    //todo count cost of startstates intervals
    std::vector<ompl::base::State * > startStates  = getStates(getMotionVectors(startMotion));

    std::vector<ompl::base::State * > goalStates  = getStates(getMotionVectors(goalMotion));

    int cost_motions1 = getCostPath(startStates);
    int cost_motions2 = getCostPath(goalStates);


    if(startStates.size()<3)
        return 0;

    int r1 = rewire(startStates);


    int r2 = rewire(goalStates);
    int cost_motions11 = getCostPath(startStates);
    int cost_motions12 =  getCostPath(goalStates);

    std::cout << "cost 1 : " << cost_motions1 << ", rewired : " << r1 << std::endl;
    std::cout << "cost 2 : " << cost_motions2 << ", rewired : " << r2 << std::endl;

    if(startStates.size() > 0){
        startMotion = getReConnectedMotions(startStates);


        std::cout << "begin" << std::endl;
        for(auto i : getStates(getMotionVectors(startMotion)))
            si_->printState(i);
        std::cout << "end" << std::endl;

        if(goalStates.size()>0)
            goalMotion = getReConnectedMotions(goalStates);
    }
    //std::vector<ompl::base::State *> newGoalStates = rewire(goalStates);

    return 0;
}

ompl::geometric::RRTnew::Motion * ompl::geometric::RRTnew::rewireMotion(Motion *startMotion,bool start_)
{

    std::vector<ompl::base::State * > startStates  = getStates(getMotionVectors(startMotion));
    /*
    int c1 = getCostPath(startStates);
    int r1 = rewire(startStates);
    */
    std::cout << "startstates : " << std::endl;
    for(auto i : startStates)
        si_->printState(i);
    std::cout<<"----------" << std::endl;


    return startMotion;

}

ompl::base::Cost ompl::geometric::RRTnew::getIncCost(Motion * m1, Motion *m2)
{
    base::Cost c_;
    //if(m1->index_changed == getChangedIndex(m1->state,m2->state)){
    if(m1->index_changed == getChangedIndexGroups(m1->state,m2->state).back()){
        c_ = opt_->combineCosts(m1->incCost, base::Cost(0.0,si_->distance(m1->state,m2->state)));
    }
    else{
        c_ = opt_->combineCosts(m1->incCost,opt_->motionCost(m1->state,m2->state));
    }
    return c_;

}

void ompl::geometric::RRTnew::printMotionCosts(Motion * z_near, Motion * z_min,Motion * z_new)
{
    std::cout << "Current z_min:-->";
    printMotion(z_min);
    std::cout<<std::endl;

    std::cout << "Current z_near-->";
    printMotion(z_near);
    std::cout<<std::endl;

    std::cout << "Current z_new-->";
    printMotion(z_new);
    std::cout << std::endl;

    base::Cost inc_cost_z_min_to_z_new = getIncCost(z_min,z_new);
    std::cout << "Min. motion cost from z_min to z_new :" << opt_->motionCost(z_min->state,z_new->state).value() <<", dist: " << opt_->motionCost(z_min->state,z_new->state).distval() << std::endl;
    std::cout << "Min. incCost from z_min to z_new: " << inc_cost_z_min_to_z_new.value() <<", dist : "<< inc_cost_z_min_to_z_new.distval();

    base::Cost inc_cost_z_near_to_z_new = getIncCost(z_near,z_new);
    std::cout << "--- Motion cost from z_near to z_new :" << opt_->motionCost(z_near->state,z_new->state).value() <<", dist: " << opt_->motionCost(z_near->state,z_new->state).distval() << std::endl;
    std::cout << "incCost from z_min to z_new: " << inc_cost_z_near_to_z_new.value() <<", dist : "<< inc_cost_z_near_to_z_new.distval();

    std::cout<<"--End printMotions"<<std::endl;
}



ompl::geometric::RRTnew::Motion *
ompl::geometric::RRTnew::chooseParent(std::vector<Motion *> Z_near, ompl::geometric::RRTnew::Motion *z_nearest,
                                      ompl::geometric::RRTnew::Motion *z_new, bool tree_inf)
{
  //  std::cout<<"---Choosing parent:" << std::endl;


    auto z_min = z_nearest;
    base::Cost c_min = z_new->incCost; // schon vorher nearest inc cost + motion cost

    for(auto z_near : Z_near)
    {
        if(validMotionCheck(tree_inf,z_near->state,z_new->state))
        {
            base::Cost motion_cost_z_min_to_z_new = opt_->motionCost(z_min->state,z_new->state);
            base::Cost motion_cost_z_near_to_z_new = opt_->motionCost(z_min->state,z_new->state);

            base::Cost inc_cost_z_near_to_z_new = getIncCost(z_near,z_new);

//            printMotionCosts(z_near,z_min,z_new);

            if(opt_->isCostBetterThan(motion_cost_z_near_to_z_new,motion_cost_z_min_to_z_new) && opt_->isCostBetterThan(inc_cost_z_near_to_z_new,c_min)) // motioncost und inccost zu near
            {
                //std::cout << "Yes its better." << std::endl;
                z_min = z_near;
                c_min = inc_cost_z_near_to_z_new;
            }
        }
    }

    return z_min;
}

void ompl::geometric::RRTnew::reConnectMotion(Motion * connectA, Motion * connectB)
{
    // set connect a s parent as b
    OMPL_DEBUG("INSIDE RECONNECT");
    connectA->parent = connectB;
    connectA->cost = opt_->motionCost(connectB->state,connectA->state);
    //connectA->index_changed = getChangedIndex(connectB->state,connectA->state);
    connectA->index_changed = getChangedIndexGroups(connectB->state,connectA->state).back();

    if(connectB->index_changed == connectA->index_changed)
    {
        connectA->incCost = opt_->combineCosts(connectB->incCost, base::Cost(0.0,connectA->cost.distval()));
    }
    else
    {
        connectA->incCost = opt_->combineCosts(connectB->incCost, connectA->cost);
    }

}

void ompl::geometric::RRTnew::rewireNew(std::vector<Motion *> Z_near, ompl::geometric::RRTnew::Motion *z_min,
                                        ompl::geometric::RRTnew::Motion *z_new, bool tree_inf)
{
    int rewired_counter = 0;
    for(auto &z_near: Z_near)
    {
        if(z_near != z_min) // oder check states
        {
            if(si_->checkMotion(z_new->state,z_near->state))
            {
                base::Cost motion_cost_z_new_to_z_near = opt_->motionCost(z_new->state,z_near->state);
                base::Cost inc_cost_z_new_to_z_near;

              //  if(z_new->index_changed == getChangedIndex(z_new->state,z_near->state))
                if(z_new->index_changed == getChangedIndexGroups(z_new->state,z_near->state).back())
                {
                    inc_cost_z_new_to_z_near = opt_->combineCosts(z_new->incCost, base::Cost(0.0,motion_cost_z_new_to_z_near.distval()));
                }else
                {
                    inc_cost_z_new_to_z_near = opt_->combineCosts(z_new->incCost, motion_cost_z_new_to_z_near);
                }

                if(opt_->isCostBetterThan(motion_cost_z_new_to_z_near, z_near->cost) &&
                   opt_->isCostBetterThan(inc_cost_z_new_to_z_near,z_near->incCost))
                {
                    //todo check if reconnection correct but cant reach here rn anyway
                    //printMotionCosts(z_near,z_min,z_new);
                    reConnectMotion(z_near,z_new);
                    rewired_counter++;
                }
            }
        }
    }
    //OMPL_DEBUG("REWIRED %d NODES", rewired_counter);
}

ompl::geometric::RRTnew::GrowState ompl::geometric::RRTnew::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                     Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nearestMotion = tree->nearest(rmotion);


    /* assume we can reach the state we go towards */
    bool reach = true;
    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nearestMotion->state, rmotion->state);
    si_->getStateSpace()->interpolate(nearestMotion->state, rmotion->state,maxDistance_, tgi.xstate);
    // statt maxdistance / d, auf maxdistance weil sonst kommt er nicht weiter

    if (si_->equalStates(nearestMotion->state, tgi.xstate))
        return TRAPPED;

    dstate = tgi.xstate;

    if(d>maxDistance_){
        reach = false;
    }

    if (si_->equalStates(nearestMotion->state, dstate))
        return TRAPPED;

/*
    if(!validMotionCheck(tgi.start,nearestMotion->state,dstate)){

        return TRAPPED;
    }
*/
  //  rewireMotion(nearestMotion,tgi.start); // ONLY PRITING THE STATES NO REWIRE JUST WRONG FUNCTION NAME TODO RENAME


    auto *x_new = createNewMotion(dstate,nearestMotion);
/*
    std::cout<< "Print x_new--> ";
    printMotion(x_new);
    std::cout<<std::endl;
*/

 /*   std::cout<<"Print nearestMotion before chooseParent:-->";
    printMotion(nearestMotion);
    std::cout<<std::endl;
*/
    std::vector<Motion * > Z_near;
    tree->nearestK(x_new,1000,Z_near); // todo k??

    auto * z_min = chooseParent(Z_near,nearestMotion,x_new,tgi.start);

  /*  std::cout <<"Chosen z_min as new parent-->";
    printMotion(z_min);
    std::cout<<std::endl;
*/
    if(!si_->equalStates(nearestMotion->state,z_min->state))
        OMPL_DEBUG("STATES NOT EQUAL");

    auto * z_new =  createNewMotion(x_new->state,z_min); // set x_new s parent to the node with min cost z_min
/*
    std::cout <<"New cost values after we reconnected x_new with z_min-->";
    printMotion(x_new);
    std::cout<<std::endl;
*/
    // in rrtstar usenewstate rejection: hier vllt check ob motion cost >1 ?
    /*if(x_new->cost.value() >1.0){
        std::cout << "x_new motion cost too high! : " << x_new->cost.value() << std::endl;
        return TRAPPED;
    }*/
    //OMPL_DEBUG("ADDING MOTION TO TREE");


    if(!validMotionCheck(tgi.start,z_min->state,z_new->state)){
        return TRAPPED;
    }
    tree->add(z_new); // wenn nicht dann f??ge erstmal hinzu
    tgi.xmotion = z_new; // what is tgi xmotion
    rewireNew(Z_near,z_min,z_new,tgi.start);



    //  std::cout <<"newcost : " << newCost.value() << std::endl;
    if(false){ // bestcost = 1.0, wenn mehr als 1 index ver??ndert wurde -> TRAPPED
        OMPL_ERROR("NO VALID MOTION FOUND EVENTHOUGH REWIRING COST =1");
        return TRAPPED;
    }


    return reach ? REACHED : ADVANCED;
}




ompl::base::PlannerStatus ompl::geometric::RRTnew::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->cost = opt_->identityCost();
        motion->incCost = opt_->identityCost();
        motion->index_changed = 0;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool solved = false;

    while (!ptc)
    {
        iterations_++;
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;
        startTree_ = !startTree_;
        TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->cost = opt_->identityCost();
                motion->index_changed = 0;
                motion->incCost = opt_->identityCost();
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree_;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;


            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root) )
            {

                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                //     std::cout<< "in SOLVED" << std::endl;

                int maxCost = getCostPath(startMotion) + getCostPath(goalMotion);
                // std::cout << "max cost : " << maxCost << std::endl;
                bestCost_ = base::Cost(maxCost);
                OMPL_DEBUG("SOLVED, FURTHER REWIRING. Current cost : %d", maxCost);

               // constructSolutionPath(startMotion,goalMotion,true,ptc); // saving path before rewiring just to plot
                /*
                startMotion = rewireMotion(startMotion);
                goalMotion = rewireMotion(goalMotion);

                bool once_flag = false;
                while (!ptc){
                    OMPL_DEBUG("SOLVED, FURTHER REWIRING. Current cost : %d", maxCost);
                    startMotion = rewireMotion(startMotion);
                    goalMotion = rewireMotion(goalMotion);

                    maxCost = getCostPath(startMotion) + getCostPath(goalMotion);

                }
                */

                constructSolutionPath(startMotion,goalMotion,false,ptc);

                solved = true;
                break;
            }
            else
            {
                //rewireTree(startMotion,goalMotion);

                bestCost_ = base::Cost(getCostPath(getStates(getMotionVectors(tgi.xmotion))));
/*
                if(true) //useRewiring
                    rewireTree(startMotion,goalMotion);
*/

                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (tgi.start)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }
        //  tgi.reverseflag = !tgi.reverseflag;


    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RRTnew::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}





/*
        if(useIsolation_ && newCost.value() == 2.0) // erlaube cost 2 sonst alg ist stuck auch wenn cube 1 ist
        {
            auto dstates = isolateStates(nmotion->state, dstate);
            std::vector<Motion *> stack_motion;
            if (dstates.size() == 0) { // konnte nichts isolieren
                std::cout<<"err "<< std::endl;
                return TRAPPED;
            }

            Motion *premotion = nmotion;
            for (auto st: dstates) {

                if (si_->equalStates(premotion->state, st))             return TRAPPED;
                if (!validMotionCheck(tgi.start, premotion->state, st)) return TRAPPED;


                auto *motion = createNewMotion(st, premotion);
                stack_motion.push_back(motion);

                premotion = motion;

            }

            for(auto const &mot_ : stack_motion){ // add motions at the end only if states could be isolated, only if loop passed without trapped
                tree->add(mot_);
                incCost = opt_->combineCosts(incCost,base::Cost(1.0)); // todo ?
            }

            tgi.xmotion = stack_motion.back();

            return reach ? REACHED : ADVANCED;
        }
       */



/*

void ompl::geometric::RRTnew::getChangedIndices(const base::State* rfrom, const base::State* rto,
                                                std::vector<int> &indices_) const{


    std::vector<double> s1,s2;
    si_->getStateSpace()->copyToReals(s1,rfrom);
    si_->getStateSpace()->copyToReals(s2,rto);

    for (size_t i= 0; i < s1.size() ; i++) {
        if (abs(s1.at(i) - s2.at(i)) > 0.0001) {
            indices_.push_back(i);
        }
    }

}

std::vector<std::vector<int>> ompl::geometric::RRTnew::getChangedGroups(const std::vector<double> &from_,const std::vector<double> &to_){
    std::vector<std::vector<int>> groups;
    for(auto group : group_indices){
        std::vector<int> gr_;
        for(auto index : group){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                gr_.push_back(index);
            }
        }
        if(gr_.size() >0)
            groups.push_back(gr_);
    }
    return groups;
}


std::vector<ompl::base::State *> ompl::geometric::RRTnew::buildIsoStates(const std::vector<double> &from_,const std::vector<double> &to_,
                                                                         std::vector<std::vector<int>> &changed_index_groups ){

    std::vector<ompl::base::State *> isolateResult;
    std::vector<double> intermediate_st(from_.size());
    intermediate_st = from_;

    for(auto v : changed_index_groups){
        for(int i : v){
            intermediate_st[i] = to_[i];
        }
        base::State *temp = si_->allocState();
        si_->getStateSpace()->copyFromReals(temp,intermediate_st);
        isolateResult.push_back(temp);
    }
    return isolateResult;
}

std::vector<std::vector<int>> ompl::geometric::RRTnew::reorderGroup(const std::vector<int> &changed_indices,
                                                                    const std::vector<std::vector<int>> &groups){

    std::vector<std::vector<int>> changed_index_groups;
    for(size_t i = 0; i < changed_indices.size() ; i++){
        std::vector<int> group;
        for(auto const group_ : groups){
            std::vector<int> group_obj;
            if((std::find(group_.begin(),group_.end(),changed_indices.at(i)) != group_.end())){
                for(size_t j = 0; j < group_.size(); j++){
                    group_obj.push_back(group_.at(j));
                }
                i+= group_.size();

                changed_index_groups.push_back(group_obj);
                break;
            }
        }
        if(i < changed_indices.size()){
            group.push_back(changed_indices.at(i));
            changed_index_groups.push_back(group);
        }
    }
    return changed_index_groups;
}

std::vector<ompl::base::State *> ompl::geometric::RRTnew::isolateStates(const base::State* rfrom, const base::State* rto){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);


    //std::vector<int> changed_indices(ompl::base::IsoManipulationOptimization::changedIndex(from_,to_,changed_indices));
    std::vector<int> changed_indices;
    getChangedIndices(rfrom,rto,changed_indices);

    std::vector<std::vector<int>> groups = getChangedGroups(from_,to_);

    std::vector<std::vector<int>> changed_index_groups = reorderGroup(changed_indices,groups);

    return buildIsoStates(from_,to_,changed_index_groups);

}
*/
