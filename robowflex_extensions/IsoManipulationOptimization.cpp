//
// Created by serboba on 10.01.22.
//

#include <robowflex_dart/IsoManipulationOptimization.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <robowflex_dart/space.h>


ompl::base::IsoManipulationOptimization::IsoManipulationOptimization(const SpaceInformationPtr &si)
        : ompl::base::OptimizationObjective(si)
{
    description_ = "Iso Manipo";

    // Setup a default cost-to-go heuristics:
  //  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);

}


std::vector<int> ompl::base::IsoManipulationOptimization::changedIndex(const std::vector<double> s1, const std::vector<double> s2){
    //
    std::vector<int> indices;
    for (size_t i= 0; i < s1.size() ; i++) {
        if (s1.at(i) != s2.at(i)) {
            indices.push_back(i);
        }
    }
    return indices;
}
ompl::base::Cost ompl::base::IsoManipulationOptimization::stateCost(const State *) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::identityCost() const {
    return Cost(0.0);
}

bool ompl::base::IsoManipulationOptimization::isCostBetterThan(Cost c1, Cost c2) const {
    if(c1.value() <= c2.value())
        return true;
    else
        return false;
}
ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCost(const State *s1, const State *s2) const
{

    double cost = 0.0;
    int nd = si_->getStateSpace()->getValueLocations().size();
    const base::StateSpacePtr &space = si_->getStateSpace();
    if(s1 == NULL || s2 == NULL)
        return infiniteCost();

    std::vector<double> s1_vals,s2_vals;
    space->copyToReals(s1_vals,s1);
    space->copyToReals(s2_vals,s2);
    for (int i= 0; i < nd ; i++){
        if(s1_vals.at(i) != s2_vals.at(i)){
         cost++;
     }

    }
   // std::cout << "cost returning : " << cost << std::endl;

    return Cost(cost);
}
/*
ompl::base::Cost ompl::base::IsoManipulationOptimization::combineCosts(Cost c1, Cost c2) const {
    if(c1.value() > c2.value())
        return c2;
    else
        return c1;
}
 */
ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCostHeuristic(const State *s1, const State *s2) const {
    return motionCost(s1,s2);
}




/*
ompl::base::Cost ompl::base::goalRegionCostToGo(const State *state, const Goal *goal) {

    const auto *goalRegion = goal->as<ompl::base::GoalStates>();
    double best_cost = 1.0;
    double goal_cost = 2.0;
    ompl::base::State *st_ = goal->getSpaceInformation()->allocState();
    auto st = st_ ->as<ompl::base::RealVectorStateSpace::StateType>();

    for(int i = 0; i < goalRegion->getStateCount(); i++){
        auto gs = goalRegion->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        double temp_cost = 0.0;
        for(int j = 0; j< goal->getSpaceInformation()->getStateDimension(); j++ ){
            if(gs->values[j] != st->values[j])
                temp_cost += 1.0;
        }
        std::cout << "temp cost to go" << std::endl;
        std::cout << temp_cost << std::endl;

        if(temp_cost < goal_cost) {
            goal_cost = temp_cost;
        }
    }
    std::cout << "goal cost, best cost" << std::endl;
    std::cout << goal_cost << "-" << best_cost << std::endl;

    return Cost(std::max(goal_cost,best_cost));

}*/
