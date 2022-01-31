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

    // Setup a default cost-to-go heuristics:
  //  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);

}

/*
void ompl::base::IsoManipulationOptimization::changedIndex(const std::vector<double> s1,
                                                                       const std::vector<double> s2,
                                                                       std::vector<int> &indices){
    for (size_t i= 0; i < s1.size() ; i++) {
        if (abs(s1.at(i) - s2.at(i)) > 0.0001) {
            indices.push_back(i);
        }
    }
}
 */
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
    std::vector<int> diff;
    diff.reserve(nd);
    for (int i= 0; i < nd ; i++){
        if(abs(s1_vals.at(i) - s2_vals.at(i)) > 0.0001){
            diff.push_back(i);

         cost++;
     }
    }

    bool flag = true;
    for(auto const group : group_indices_){
        int size_counter = 0;
        for(auto const index : group){
            if(std::find(diff.begin(),diff.end(),index) != diff.end()){
                size_counter++;
            }
            if (size_counter == group.size() && size_counter > 1)
                cost -= 1.0;
        }
    }

/*
    if(flag) {
        cost -=1.0;
    }
*/
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
    // if only group was changed

    return motionCost(s1,s2);
}



std::vector<std::vector<int>> ompl::base::IsoManipulationOptimization::getGroupIndices() {
    return group_indices_;
}



