//
// Created by serboba on 10.01.22.
//

#include <robowflex_dart/IsoManipulationOptimization.h>


ompl::base::IsoManipulationOptimization::IsoManipulationOptimization(const SpaceInformationPtr &si)
        : ompl::base::OptimizationObjective(si)
{
    description_ = "Iso Manipo";

    // Setup a default cost-to-go heuristics:
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::stateCost(const State *) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCost(const State *s1, const State *s2) const
{

    int cost = 0;
    int nd = si_->getStateSpace()->getValueLocations().size();
    const base::StateSpacePtr &space = si_->getStateSpace();
    std::vector<double> s1_vals,s2_vals;
    space->copyToReals(s1_vals,s1);
    space->copyToReals(s2_vals,s2);
    for (int i= 0; i < nd ; i++){
     if(s1_vals.at(i) != s2_vals.at(i)){
         cost++;
     }

    }
   // std::cout << "cost returning : " << cost << std::endl;
    //return Cost(andere state anzahl)
    return Cost(cost);
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCostHeuristic(const State *s1, const State *s2) const {
    return motionCost(s1,s2);
}