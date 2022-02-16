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

ompl::base::Cost ompl::base::IsoManipulationOptimization::identityCost() const
{
    return Cost(0.0,0.0);
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::infiniteCost() const
{
    return Cost(std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity());
}

bool ompl::base::IsoManipulationOptimization::isCostBetterThan(const Cost &c1, const Cost &c2) const
{
/*
    if(c1.value() == c2.value())// action cost equal
        return c1.distval() < c2.distval();
    else
        return c1.value() < c2.value();
*/

    if(c1.value() < c2.value()){
        return c1.distval() < c2.distval();
    }
    return false;

/*
    if(c1.value() == 1 && c2.value() == 1){
        return c1.distval() < c2.distval();
    }

    else if(c1.value() < c2.value()){
        return c1.distval() < c2.distval();
    }
        return false;
*/
}

bool ompl::base::IsoManipulationOptimization::isCostEquivalentTo(const Cost &c1, const Cost &c2) const
{
    if (c1.value() == c2.value())
        if (abs(c1.distval() -c2.distval())< 1e-10)
            return true;
    return false;
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::combineCosts(const Cost &c1, const Cost &c2) const
{
    return Cost(c1.value()+c2.value(),c1.distval()+c2.distval());

}

ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCost(const State *s1, const State *s2) const
{

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

    return Cost(double(action_cost),si_->distance(s1,s2));
}


ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCostHeuristic(const State *s1, const State *s2) const {
    // if only group was changed

    return motionCost(s1,s2);
}






