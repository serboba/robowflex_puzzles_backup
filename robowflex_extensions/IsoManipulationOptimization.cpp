//
// Created by serboba on 10.01.22.
//

#include <robowflex_dart/IsoManipulationOptimization.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <robowflex_dart/space.h>


ompl::base::IsoManipulationOptimization::IsoManipulationOptimization(const SpaceInformationPtr &si, std::vector<std::vector<int>> group_indices)
        : ompl::base::OptimizationObjective(si), group_indices_(group_indices)
{
    description_ = "Iso Manipo";


}

ompl::base::Cost ompl::base::IsoManipulationOptimization::stateCost(const State *) const
{
    return identityCost();
}


ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCost(const State *s1, const State *s2) const {

    if(s1 == NULL || s2 == NULL) {
        return infiniteCost();
    }
    int action_cost = 0;
    const base::StateSpacePtr &space = si_->getStateSpace();
    std::vector<double> s1_vals,s2_vals;
    space->copyToReals(s1_vals,s1);
    space->copyToReals(s2_vals,s2);

    for(auto const &group: group_indices_){
        for(auto const &index_in_gr : group){
            if(abs(s1_vals.at(index_in_gr) - s2_vals.at(index_in_gr)) > 1e-10){
                action_cost++;
                break;
            }
        }
    }

    return Cost(double(action_cost));
}


ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCostHeuristic(const State *s1, const State *s2) const {
    // if only group was changed

    return motionCost(s1,s2);
}






