/*


ompl::base::Cost ompl::base::IsoManipulationOptimization::identityCost() const {

    return DistanceAndActionCost(0,0.0);
}

bool ompl::base::IsoManipulationOptimization::isCostBetterThan(DistanceAndActionCost c1,            // c1 is better than c2
                                                   DistanceAndActionCost c2) const {
    if(c1.dim_ == c2.dim_){
        if(c1.distance_ <= c2.distance_){
            return true;
        }
    }else if(c1.dim_ < c2.dim_){
        return true;
    }

    return false;
}

bool ompl::base::IsoManipulationOptimization::isCostEquivalentTo(DistanceAndActionCost c1,
                                                     DistanceAndActionCost c2) const {
    if(c1.dim_ == c2.dim_){
        if(abs(c1.distance_-c2.distance_) < DBL_EPSILON)
            return true;
    }
    return false;
}

ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCost(const State *s1, const State *s2) const
{

    int action_cost = 0;
    int nd = si_->getStateSpace()->getValueLocations().size();

    const base::StateSpacePtr &space = si_->getStateSpace();
    if(s1 == NULL || s2 == NULL)
        return DistanceAndActionCost(std::numeric_limits<int>::infinity(),std::numeric_limits<double>::infinity()) ;

    std::vector<double> s1_vals,s2_vals;
    space->copyToReals(s1_vals,s1);
    space->copyToReals(s2_vals,s2);
    std::vector<int> diff;
    diff.reserve(nd);
    for (int i= 0; i < nd ; i++){
        if(abs(s1_vals.at(i) - s2_vals.at(i)) > DBL_EPSILON){
            diff.push_back(i);

            action_cost++;
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
                action_cost -= 1.0;
        }
    }

    return DistanceAndActionCost(action_cost,si_->distance(s1,s2));
}


ompl::base::Cost ompl::base::IsoManipulationOptimization::motionCostHeuristic(const State *s1, const State *s2) const {
    // if only group was changed

    return motionCost(s1,s2);
}







 */