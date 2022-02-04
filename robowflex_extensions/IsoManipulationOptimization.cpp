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

/*
ompl::base::Cost ompl::base::IsoManipulationOptimization::infiniteCost() const
{
    return Cost(std::numeric_limits<double>::infinity(),std::numeric_limits<int>::infinity());
}
*/
/*
bool ompl::base::IsoManipulationOptimization::isCostBetterThan(const base::Cost &c1_, const base::Cost &c2_) const { // c1 is better than c2

    if(c1_.value() >2)
        return false;
    else
        return (c1_.value() <= c2_.value());

}
*/
ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCost(const State *s1, const State *s2) const {

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

/*
bool ompl::base::IsoManipulationOptimization::isCostEquivalentTo(const base::Cost &c1_, const base::Cost &c2_) const {

 if(c1_.action() == c2_.action() && abs(c1_.value()-c2_.value())< 1e-8)
       return true;
   else
       return false;

}
*/
/*
ompl::base::Cost ompl::base::IsoManipulationOptimization::combineCosts(const base::Cost &c1,const base::Cost &c2) const {

  //  return Cost((c1.value()+c2.value()),std::max(c1.action(),c2.action()));
    return Cost(std::max(c1.value(),c2.value()));
}
*/
ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCostHeuristic(const State *s1, const State *s2) const {
    // if only group was changed

    return motionCost(s1,s2);
}
/*
ompl::base::Cost ompl::base::IsoManipulationOptimization::identityCost() const {
    return Cost(0.0,group_indices_.size());
}
*/






