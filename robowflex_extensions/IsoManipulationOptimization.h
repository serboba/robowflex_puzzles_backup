//
// Created by serboba on 10.01.22.
//

#ifndef ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H
#define ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H


#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <robowflex_dart/DistanceAndActionCost.h>

namespace ompl
{
    namespace base


    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class IsoManipulationOptimization : public OptimizationObjective
        {
        public:
            IsoManipulationOptimization(const SpaceInformationPtr &si, std::vector<std::vector<int>> group_indices);

            Cost stateCost(const State *s) const override;

        //    Cost identityCost() const override;

            Cost motionCost(const State *s1, const State *s2) const override;

            Cost motionCostHeuristic(const State *s1, const State *s2) const override;


            //Cost goalRegionCostToGo(const State *state, const Goal *goal) ;

        //    bool isCostBetterThan(const base::Cost &c1,const base::Cost &c2) const override;

        //    Cost infiniteCost() const override;
        //    bool isCostEquivalentTo(const base::Cost &c1_, const base::Cost &c2_) const override;
     //       Cost combineCosts(const base::Cost &c1,const base::Cost &c2) const override;

        protected:

            std::vector<std::vector<int>> group_indices_;
        };
    }
}

#endif //ROBOWFLEX_DART_ISOMANIPULATIONOPTIMIZATION_H

