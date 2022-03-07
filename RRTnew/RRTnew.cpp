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
    addPlannerProgressProperty("best cost DOUBLE", [this] { return bestCostProgressProperty(); });
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
    incCost = opt_->infiniteCost();
}

void ompl::geometric::RRTnew::freeMemory()
{
    std::vector<Motion *> motions;
    int counter = 0;
    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr){}
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



void ompl::geometric::RRTnew::buildIsoStates(const std::vector<double> &from_,const std::vector<double> &to_,
                                                                         std::vector<int> &changed_index_groups,
                                                                         std::vector<ompl::base::State* > &iso_ ){

    std::vector<ompl::base::State *> isolateResult;
    std::vector<double> intermediate_st(from_.size());
    intermediate_st = from_;

    for (size_t v = 0; v < changed_index_groups.size(); v++) {
        for (int i: group_indices.at(changed_index_groups.at(v))) {
            intermediate_st[i] = to_[i];
        }
        base::State *temp = si_->allocState();
        si_->getStateSpace()->copyFromReals(temp, intermediate_st);
        iso_.push_back(temp);
    }
}

std::vector<int> ompl::geometric::RRTnew::getChangedGroups(const std::vector<double> &from_,const std::vector<double> &to_){
    std::vector<int> groups;
    for(size_t i = 0 ; i < group_indices.size(); i++){
        for(auto index : group_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                groups.push_back(i);
                break;
            }
        }
    }
    return groups;
}

std::vector<int> ompl::geometric::RRTnew::getChangedGroups(const base::State* rfrom, const base::State* rto){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);

    std::vector<int> groups;
    for(size_t i = 0 ; i < group_indices.size(); i++){
        for(auto index : group_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 1e-10){
                groups.push_back(i);
                break;
            }
        }
    }

    return groups;
}


void ompl::geometric::RRTnew::isolateStates(const base::State* rfrom, const base::State* rto, std::vector<ompl::base::State*> &iso_){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);

    std::vector<int> groups = getChangedGroups(from_,to_);

    buildIsoStates(from_,to_,groups,iso_);

}

void ompl::geometric::RRTnew::createNewMotion(const base::State *st, ompl::geometric::RRTnew::Motion *premotion,
                                              ompl::geometric::RRTnew::Motion *newmotion){
    si_->copyState(newmotion->state, st);
    newmotion->parent = premotion;
    newmotion->root = premotion->root;
    newmotion->cost = opt_->motionCost(premotion->state,st);
    newmotion->index_changed = getChangedIndex(premotion->state,st);
}




bool ompl::geometric::RRTnew::validMotionCheck(const bool start, const base::State *from_,const base::State *to_){
    bool validmo= start ? si_->checkMotion(from_, to_) :
                  si_->isValid(to_) && si_->checkMotion(to_, from_);

    return validmo;

}

void ompl::geometric::RRTnew::getMotionVectors(Motion * mot_,std::vector<Motion *> &vec)
{
    /* construct the motion vec */
    Motion *solution = mot_;
    while (solution != nullptr)
    {
        vec.push_back(solution);
        solution = solution->parent;
    }

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
    OMPL_ERROR("NOTHING FOUND");
    std::cout<< "nothing found" << std::endl;
    si_->printState(from);
    si_->printState(to);
    return -1;
}


std::vector<ompl::base::State * > ompl::geometric::RRTnew::getStates(std::vector<ompl::geometric::RRTnew::Motion *> motions)
{
    std::vector<ompl::base::State * > states;
    states.reserve(motions.size());

    for(auto &i: motions){
        if(!states.empty() && si_->equalStates(i->state,states.back())){
            continue;
        }
        states.push_back(i->state);
    }

    return states;
}


void ompl::geometric::RRTnew::reConnect(ompl::base::State *from,
                                        std::vector<std::pair<ompl::base::State *,int >> &queue_,
                                        std::vector<ompl::base::State * > &rewireResult )
{
    for(size_t i = 0; i < queue_.size(); i++)
    {
        ompl::base::State * newEdge = si_->allocState();
        getIntermediateState(from,queue_.at(i).first,newEdge,queue_.at(i).second);
        if(!si_->checkMotion(from,newEdge))
        {
            si_->freeState(newEdge);
            if(rewireResult.size()>1)
            {
                for(auto &st : rewireResult)
                    si_->freeState(st);
                rewireResult.clear();
            }
            std::vector<ompl::base::State *>().swap(rewireResult); // free mem
            return;
        }

        rewireResult.push_back(newEdge);
        from = newEdge;
    }
}


void ompl::geometric::RRTnew::reConnect(ompl::base::State *from, std::vector<std::pair<ompl::base::State *,int >> &prio_,
                                                                     std::vector<std::pair<ompl::base::State *,int >> &stack_,
                                                                     std::vector<ompl::base::State *> &rewireResult)
{
    reConnect(from,prio_,rewireResult);
    if(!rewireResult.empty())
        reConnect(rewireResult.back(),stack_,rewireResult);

    if(rewireResult.empty())
        rewireResult.clear();
    else
    {
        for(auto &a : prio_)
            si_->freeState(a.first);

        for(auto &a : stack_)
            si_->freeState(a.first);

        prio_.clear();
        stack_.clear();
    }
}


int ompl::geometric::RRTnew::pathDefrag(std::vector<ompl::base::State *> &mainPath) {

    int rewireCount = 0;
    int prev_index = getChangedIndex(mainPath.at(0),mainPath.at(1));

    for(size_t toID = 1; toID < mainPath.size()-1; ++toID){
        size_t fromID = toID-1;

        ompl::base::State *fromState = mainPath.at(fromID);
        ompl::base::State  *toState = mainPath.at(toID);

        if(getChangedIndex(fromState,toState) != prev_index){
            size_t subsearch_id = toID;
            ompl::base::State *subFrom = mainPath.at(subsearch_id-1);
            ompl::base::State  *subTo = mainPath.at(subsearch_id);

            std::vector<std::pair<ompl::base::State *,int>> mergeIndices;
            std::vector<std::pair<ompl::base::State *,int>> queueIndices;
            queueIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));

            while (getChangedIndex(subTo,subFrom) != prev_index && subsearch_id < mainPath.size()-1)
            {
                subsearch_id++;

                subFrom = mainPath.at(subsearch_id-1);
                subTo = mainPath.at(subsearch_id);

                if(getChangedIndex(subFrom,subTo) != prev_index)
                    queueIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));
                else
                { //start same index again done
                    if(subsearch_id >= mainPath.size()-1) // this is the case when we re already at the end of the path, last state with diff index
                        mergeIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));

                    break;
                }
            }

            while(getChangedIndex(subTo,subFrom) == prev_index && subsearch_id < mainPath.size()-1)
            {
                subFrom = mainPath.at(subsearch_id-1);
                subTo = mainPath.at(subsearch_id);

                if(getChangedIndex(subFrom,subTo) == prev_index)
                    mergeIndices.push_back(std::make_pair(subTo,getChangedIndex(subFrom,subTo)));
                else
                    break;
                subsearch_id++;
            }

            if(subsearch_id >= mainPath.size())
                OMPL_ERROR("SUB SEARCH ID OVER SIZE ERR");

            std::vector<ompl::base::State * > rewiredConnection;

            if(!mergeIndices.empty() && !queueIndices.empty())
                reConnect(fromState,mergeIndices,queueIndices,rewiredConnection);
            else{
                if(toID>= mainPath.size()-1)
                    return rewireCount;
                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            if(rewiredConnection.empty()){ // no rewiring possible because of invalid motion but no problem, just start with next the next state and go on
                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            rewireCount+=rewiredConnection.size();
            int l = 0;

            //TODO FREE REWIRE FAILURE OR REPLACED PATH
            int test2 = toID + rewiredConnection.size();
            int behind_index = mainPath.size()-test2;

            mainPath.erase(mainPath.begin()+toID, mainPath.end()-behind_index);

            for(size_t j = toID; j <toID+rewiredConnection.size(); j++){
                mainPath.insert(mainPath.begin()+j,rewiredConnection.at(l));
                l++;
            }

            fromID = fromID+mergeIndices.size(); //
            if(toID>= mainPath.size()-1)
                return rewireCount;
            prev_index = getChangedIndex(mainPath.at(fromID),mainPath.at(fromID+1));
            toID = fromID+1;


        }
    }

    return rewireCount;

}

int ompl::geometric::RRTnew::getCostPath(std::vector<ompl::base::State * > &states_)
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


void ompl::geometric::RRTnew::simplifyActionIntervals(std::vector<ompl::base::State*> &mainPath) // finds intervals of same action index, builds shortcuts
{
    int prev_index = getChangedIndex(mainPath.at(0),mainPath.at(1));
    std::vector<int> transitions;
    std::vector<std::pair<int,int>> intervals; // pair of start and end
    for (size_t i = 1; i < mainPath.size()-1 ; ++i) {

        if(prev_index != getChangedIndex(mainPath.at(i),mainPath.at(i+1)))
        {
            transitions.push_back(i);
            prev_index = getChangedIndex(mainPath.at(i),mainPath.at(i+1));
        }
        else
        {
            si_->freeState(mainPath.at(i));
        }
    }
    std::vector<ompl::base::State *> simplifiedPath;
    simplifiedPath.push_back(mainPath.at(0));
    for(auto &index : transitions)
    {
        if(!si_->equalStates(mainPath.at(index),simplifiedPath.back()))
            simplifiedPath.push_back(mainPath.at(index));
    }
    simplifiedPath.push_back(mainPath.back());


    for(size_t i = simplifiedPath.size()-1 ; i > 0 ; i--)
    {
        if(getChangedIndex(simplifiedPath.at(i),simplifiedPath.at(i-1)) != 0) // goal group 0, fetch 4 // CUT AFTER GOAL REACHED
        {
            si_->freeState(simplifiedPath.at(i));
            simplifiedPath.erase(simplifiedPath.begin()+i);
        } else
        {
            break;
        }
    }
    mainPath.clear();

    std::vector<ompl::base::State *>().swap(mainPath);
    mainPath = simplifiedPath;

}


ompl::geometric::RRTnew::GrowState ompl::geometric::RRTnew::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                     Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;
    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    si_->getStateSpace()->interpolate(nmotion->state, rmotion->state,maxDistance_, tgi.xstate);
    // statt maxdistance / d, auf maxdistance weil sonst kommt er nicht weiter

    if (si_->equalStates(nmotion->state, tgi.xstate))
        return TRAPPED;

    dstate = tgi.xstate;

    if(d>maxDistance_){
        reach = false;
    }

    if (si_->equalStates(nmotion->state, dstate))
        return TRAPPED;


    if(!validMotionCheck(tgi.start,nmotion->state,dstate)){

        return TRAPPED;
    }

    auto newCost = opt_->motionCost(nmotion->state,dstate);

    if(newCost.value()  > 1.0){ // bestcost = 1.0, wenn mehr als 1 index verändert wurde -> TRAPPED
        if(useIsolation_ ) // erlaube cost 2 sonst alg ist stuck auch wenn cube 1 ist
        {
            std::vector<ompl::base::State *> dstates;

            isolateStates(nmotion->state, dstate,dstates);
            std::vector<Motion *> stack_motion;
            if (dstates.size() == 0) { // konnte nichts isolieren
                return TRAPPED;
            }

            Motion *premotion = nmotion;
            for (auto st: dstates) { // TODO FEHLER HIER

                if (si_->equalStates(premotion->state, st))
                    return TRAPPED;
                if (!validMotionCheck(tgi.start, premotion->state, st))
                {
                    for(auto &fr: dstates){
                        si_->freeState(fr);
                    }
                    dstates.clear();
                    std::vector<ompl::base::State *>().swap(dstates); // free mem

                    for(auto &m: stack_motion)
                    {
                        si_->freeState(m->state);
                    }
                    stack_motion.clear();

                    return TRAPPED;
                }

                auto *motion = new Motion(si_);
                createNewMotion(st, premotion,motion);

                stack_motion.push_back(motion);
                premotion = motion;

            }

            for (auto const &mot_: stack_motion) { // add motions at the end only if states could be isolated, only if loop passed without trapped
                tree->add(mot_);
                incCost = opt_->combineCosts(incCost, base::Cost(1.0)); // todo ?
            }

            tgi.xmotion = stack_motion.back();

            return reach ? REACHED : ADVANCED;
        }
        return TRAPPED;
    }

    incCost = opt_->combineCosts(incCost,opt_->motionCost(nmotion->state,dstate )); // todo
    auto * motion = new Motion(si_);
    createNewMotion(dstate,nmotion,motion);
    tree->add(motion);
    tgi.xmotion = motion;

    return reach ? REACHED : ADVANCED;
}

void ompl::geometric::RRTnew::simplifyPath(std::vector<ompl::base::State *> &path)
{

    int counter =0;
    bool count_flag = true;
    base::Cost maxCost_(getCostPath(path));
    std::vector<ompl::base::State *> path_temp = path;

    while(count_flag){
        pathDefrag(path_temp);
        base::Cost temp_cost(getCostPath(path_temp));
         if(temp_cost.value() <= maxCost_.value())
        {
            if(temp_cost.value() == maxCost_.value())
                counter++;
            if(counter ==2)
                count_flag = false;
            maxCost_ = temp_cost;

            path = path_temp;
        }
        else
        {
            break;
        }
    }

    simplifyActionIntervals(path);

}


void ompl::geometric::RRTnew::checkRepairPath(std::vector<ompl::base::State *> &path_)
{

    for(size_t i = 0; i < path_.size()-1; i++)
    {
        if(si_->equalStates(path_.at(i),path_.at(i+1))){  // equal states may be added somewhere else (outside growstate, i couldnt find where)
            path_.erase(path_.begin()+i);
        }

        if(opt_->motionCost(path_.at(i),path_.at(i+1)).value()> 1.0) // path defrag needs every state with motion cost 1.0
         {
             std::vector<ompl::base::State *> iso_;
             isolateStates(path_.at(i),path_.at(i+1),iso_);
             bool check = true;
             if(check){
                 path_.insert(path_.begin()+i+1,iso_.begin(),iso_.end());
                 //todo check collision if yes new interpolate?
             }

         }

    }

}

void ompl::geometric::RRTnew::constructSolutionPath(ompl::geometric::PathGeometric &path, Motion * startMotion, Motion * goalMotion)
{
    if (startMotion->parent != nullptr)
        startMotion = startMotion->parent;
    else
        goalMotion = goalMotion->parent;

    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

    std::vector<Motion *> mpath1;
    getMotionVectors(startMotion,mpath1);
    std::vector<Motion *> mpath2;
    getMotionVectors(goalMotion,mpath2);


    path.getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i){
        path.append(mpath1[i]->state);
    }
    for (auto &i : mpath2){
        path.append(i->state);
    }

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

    auto best_path(std::make_shared<PathGeometric>(si_));
    bestCost_ = opt_->infiniteCost();


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

                auto path(std::make_shared<PathGeometric>(si_));
                constructSolutionPath(*path,startMotion,goalMotion);

                checkRepairPath(path->getStates());

                simplifyPath(path->getStates());

                if(getCostPath(path->getStates()) < bestCost_.value())
                {
           //         std::cout << "we found better path :" << getCostPath(path->getStates()) << ", earlier : " << bestCost_.value() << std::endl;
                    std::vector<ompl::base::State *>().swap(best_path->getStates()); // free mem
                    best_path = path;
                    bestCost_ = base::Cost(getCostPath(best_path->getStates()),0.0);
                }else
                {
                    std::vector<ompl::base::State *>().swap(path->getStates()); // free mem
                }

                if(ptc)
                {
                    pdef_->addSolutionPath(best_path, false, 0.0, getName());
                    solved = true;
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                if(ptc && best_path->getStates().size() > 1)
                {
                    pdef_->addSolutionPath(best_path, false, 0.0, getName());
                    solved = true;
                    break;

                }
                else if (tgi.start)
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

std::string ompl::geometric::RRTnew::bestCostProgressProperty() const
{
    return std::to_string(this->bestCost_.value());
}
